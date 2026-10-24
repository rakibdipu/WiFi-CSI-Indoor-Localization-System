/* ==========================================================================
 * main.c — Application entry point
 *
 * Sequence of events:
 *
 *   1. NVS init
 *   2. LED init (optional visual feedback)
 *   3. TCP/IP netif + default event loop init
 *   4. Wi-Fi station init → connect to phone hotspot
 *   5. Wait for IP (blocking, with retry logic)
 *   6. csi_init()  — create queue + processing task
 *   7. csi_start() — register callback, apply config, enable CSI
 *   8. traffic_gen_task — sends UDP to gateway → generates downlink frames
 *                         from the phone AP → triggers CSI callback
 *
 * The only output going to USB-UART is:
 *   • Lines starting with "CSI_DATA," → parsed by csi_receiver.py
 *   • Lines starting with "STATS,"    → informational; ignored by parser
 *   • ESP_LOGx lines                  → printed at WARN level and above only
 *
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "driver/gpio.h"

#include "config.h"
#include "wifi_csi.h"

/* --------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------*/
static const char *TAG = "main";

/* FreeRTOS event group bits for Wi-Fi connection state */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* --------------------------------------------------------------------------
 * Module-level state
 * --------------------------------------------------------------------------*/
static EventGroupHandle_t s_wifi_event_group = NULL;
static int                s_retry_count      = 0;
static esp_ip4_addr_t     s_gateway_ip       = { .addr = 0 };

/* ==========================================================================
 * LED helpers (optional)
 * ==========================================================================*/
static void led_init(void)
{
#if STATUS_LED_GPIO >= 0
    gpio_reset_pin((gpio_num_t)STATUS_LED_GPIO);
    gpio_set_direction((gpio_num_t)STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)STATUS_LED_GPIO, 0);
#endif
}

static inline void led_set(int state)
{
#if STATUS_LED_GPIO >= 0
    gpio_set_level((gpio_num_t)STATUS_LED_GPIO, state ? 1 : 0);
#else
    (void)state;
#endif
}

/* ==========================================================================
 * Wi-Fi event handler
 * ==========================================================================*/
static void wifi_event_handler(void       *arg,
                               esp_event_base_t event_base,
                               int32_t    event_id,
                               void       *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* Station started: attempt connection -------------------------------- */
        ESP_LOGI(TAG, "Wi-Fi station started, connecting to \"%s\"...", WIFI_SSID);
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Disconnected: retry up to WIFI_MAX_RETRY times -------------------- */
        led_set(0);
        wifi_event_sta_disconnected_t *disc =
            (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected (reason=%d), retry %d/%d",
                 disc->reason, s_retry_count + 1, WIFI_MAX_RETRY);

        if (s_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Max retries reached. Check SSID/password in config.h");
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /* Got IP address ---------------------------------------------------- */
        ip_event_got_ip_t *ip_evt = (ip_event_got_ip_t *)event_data;
        s_gateway_ip  = ip_evt->ip_info.gw;
        s_retry_count = 0;
        led_set(1);

        ESP_LOGI(TAG, "Connected! IP=" IPSTR "  Gateway=" IPSTR,
                 IP2STR(&ip_evt->ip_info.ip),
                 IP2STR(&s_gateway_ip));

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ==========================================================================
 * wifi_sta_connect()
 *
 * Initialises the Wi-Fi stack as a station, registers events, and blocks
 * until the connection is established or fails permanently.
 *
 * Returns true on success, false on failure.
 * ==========================================================================*/
static bool wifi_sta_connect(void)
{
    s_wifi_event_group = xEventGroupCreate();

    /* Initialise TCP/IP adapter and default Wi-Fi netif -------------------- */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* Wi-Fi driver init with default config --------------------------------- */
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    /* Register event handlers ----------------------------------------------- */
    esp_event_handler_instance_t inst_any_id;
    esp_event_handler_instance_t inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &inst_got_ip));

    /* Configure station ---------------------------------------------------- */
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid              = WIFI_SSID,
            .password          = WIFI_PASSWORD,
            /* Use PMF (Protected Management Frames) where possible          */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable  = true,
                .required = false,
            },
        },
    };

    /* Optionally use open networks (no password) ---------------------------- */
    if (strlen(WIFI_PASSWORD) == 0) {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");

    /* Block until connected or failed --------------------------------------- */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,            /* Don't clear bits on exit */
        pdFALSE,            /* Wait for any one bit */
        pdMS_TO_TICKS(30000) /* 30-second timeout */
    );

    bool connected = (bits & WIFI_CONNECTED_BIT) != 0;

    /* Unregister handlers (no longer needed after connection) --------------- */
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, inst_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, inst_any_id);
    vEventGroupDelete(s_wifi_event_group);

    return connected;
}

/* ==========================================================================
 * Traffic generation task
 *
 * Sends small UDP datagrams to the phone gateway IP.
 * The phone's 802.11 MAC automatically sends an ACK frame for each received
 * data frame.  Those ACK frames are received by the ESP32, which triggers
 * the CSI callback — providing a steady, controlled-rate CSI stream.
 *
 * Sending at TRAFFIC_GEN_INTERVAL_MS = 10 ms → ~100 UDP/sec →
 * ~100 MAC-layer ACKs/sec → ~100 CSI callbacks/sec.
 * Plus beacon frames (~10/sec) from the phone → total ~110 CSI packets/sec.
 * ==========================================================================*/
static void traffic_gen_task(void *arg)
{
    /* Destination: phone (gateway) IP, arbitrary port ---------------------- */
    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(TRAFFIC_GEN_DST_PORT),
        .sin_addr.s_addr = s_gateway_ip.addr,
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "traffic_gen: socket() failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    /* Small payload — content doesn't matter; we only need 802.11 ACKs     */
    const char payload[] = "csi";
    TickType_t  last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TRAFFIC_GEN_INTERVAL_MS);

    ESP_LOGI(TAG, "Traffic generator started → " IPSTR ":%d @ %d ms interval",
             IP2STR(&s_gateway_ip), TRAFFIC_GEN_DST_PORT,
             TRAFFIC_GEN_INTERVAL_MS);

    while (1) {
        sendto(sock, payload, sizeof(payload) - 1, 0,
               (struct sockaddr *)&dest, sizeof(dest));
        vTaskDelayUntil(&last_wake, period);
    }

    /* Unreachable — but good practice */
    close(sock);
    vTaskDelete(NULL);
}

/* ==========================================================================
 * app_main
 * ==========================================================================*/
void app_main(void)
{
    /* Print a clear boot banner to help identify firmware version          */
    printf("\n");
    printf("========================================\n");
    printf("  Wi-Fi CSI Node — Phase 1\n");
    printf("  Board  : ESP32-S3-CAM N16R8\n");
    printf("  Target : %s\n", WIFI_SSID);
    printf("  Output : CSI_DATA,... lines on UART\n");
    printf("========================================\n");
    fflush(stdout);

    /* 1. LED ---------------------------------------------------------------- */
    led_init();

    /* 2. Non-Volatile Storage — required by Wi-Fi driver ------------------- */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased and re-initialised");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* 3. Initialise CSI subsystem (creates queue + tasks) before Wi-Fi ----- */
    csi_init();

    /* 4. Connect to phone hotspot ------------------------------------------ */
    if (!wifi_sta_connect()) {
        ESP_LOGE(TAG, "Wi-Fi connection FAILED.\n"
                      "  → Check WIFI_SSID and WIFI_PASSWORD in config.h\n"
                      "  → Make sure your phone hotspot is ON\n"
                      "  → Reflash after editing config.h");

        /* Blink LED rapidly to indicate error */
        while (1) {
            led_set(1); vTaskDelay(pdMS_TO_TICKS(100));
            led_set(0); vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    /* 5. Enable CSI capture ------------------------------------------------ */
    csi_start();

    /* 6. Start traffic generator to ensure steady CSI packet rate ---------- */
    xTaskCreatePinnedToCore(
        traffic_gen_task,
        "traffic_gen",
        TRAFFIC_GEN_TASK_STACK,
        NULL,
        TRAFFIC_GEN_TASK_PRIO,
        NULL,
        0   /* Core 0 — same core as Wi-Fi stack is fine for net sockets */
    );

    /* 7. Print startup notice then let tasks run forever ------------------- */
    ESP_LOGI(TAG, "CSI streaming started. Reading on laptop with csi_receiver.py");

    /* app_main can return — FreeRTOS scheduler continues running tasks.     */
}
