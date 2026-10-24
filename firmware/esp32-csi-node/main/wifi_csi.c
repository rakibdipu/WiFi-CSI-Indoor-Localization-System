/* ==========================================================================
 * wifi_csi.c — CSI acquisition and processing subsystem
 *
 * Architecture:
 *
 *   Wi-Fi ISR context
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │  wifi_csi_cb()  ← called by Wi-Fi task at HIGH priority        │
 *   │    • copies wifi_csi_info_t fields into csi_packet_t            │
 *   │    • memcpy the I/Q buffer                                       │
 *   │    • xQueueSendFromISR() → non-blocking, drops if queue full    │
 *   └───────────────────────┬─────────────────────────────────────────┘
 *                           │  FreeRTOS queue (CSI_QUEUE_SIZE slots)
 *   ┌───────────────────────▼─────────────────────────────────────────┐
 *   │  csi_process_task()  ← LOW priority, Core 1                    │
 *   │    • xQueueReceive() — blocks until packet arrives              │
 *   │    • skips first_word_invalid bytes                              │
 *   │    • formats CSV line                                            │
 *   │    • fwrite() to stdout (→ UART0 → USB-UART chip → laptop)     │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * Output line format (one line per CSI packet):
 *   CSI_DATA,<timestamp_us>,<rssi>,<noise_floor>,<channel>,<sec_ch>,
 *            <bandwidth>,<sig_mode>,<antenna>,<num_pairs>,
 *            I0,Q0,I1,Q1,...,In,Qn\n
 *
 * ==========================================================================*/

#include "wifi_csi.h"
#include "config.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "wifi_csi";

/* Internal state ---------------------------------------------------------- */
static QueueHandle_t s_csi_queue    = NULL;
static volatile uint32_t s_pkt_count = 0;

/* ==========================================================================
 * Fast integer-to-decimal helper
 *
 * Writes the decimal ASCII representation of a signed 8-bit integer
 * into buf[].  Returns the number of characters written (no NUL terminator).
 * Faster than snprintf() for this hot path.
 * ==========================================================================*/
static inline int int8_to_str(char *buf, int8_t value)
{
    int pos = 0;
    int v = (int)value;

    if (v < 0) {
        buf[pos++] = '-';
        v = -v;
    }

    if (v >= 100) {
        buf[pos++] = (char)('0' + v / 100);
        v %= 100;
        buf[pos++] = (char)('0' + v / 10);
        v %= 10;
        buf[pos++] = (char)('0' + v);
    } else if (v >= 10) {
        buf[pos++] = (char)('0' + v / 10);
        v %= 10;
        buf[pos++] = (char)('0' + v);
    } else {
        buf[pos++] = (char)('0' + v);
    }

    return pos;
}

/* ==========================================================================
 * CSI callback — called from the Wi-Fi task (high priority / ISR context)
 *
 * Rules:
 *   • MUST NOT block (no vTaskDelay, no mutex wait, no malloc)
 *   • MUST copy buf[] before returning (memory is freed after return)
 *   • Use xQueueSendFromISR() — not xQueueSend()
 * ==========================================================================*/
static void IRAM_ATTR wifi_csi_cb(void *ctx, wifi_csi_info_t *csi_info)
{
    if (csi_info == NULL || csi_info->buf == NULL || csi_info->len == 0) {
        return;
    }

    csi_packet_t pkt;

    /* Metadata from rx_ctrl struct ---------------------------------------- */
    pkt.timestamp_us      = esp_timer_get_time();
    pkt.rssi              = csi_info->rx_ctrl.rssi;
    pkt.noise_floor       = csi_info->rx_ctrl.noise_floor;
    pkt.channel           = csi_info->rx_ctrl.channel;
    pkt.secondary_channel = csi_info->rx_ctrl.secondary_channel;
    pkt.sig_mode          = csi_info->rx_ctrl.sig_mode;
    pkt.bandwidth         = csi_info->rx_ctrl.cwb;
    pkt.antenna           = csi_info->rx_ctrl.ant;
    pkt.first_word_invalid = csi_info->first_word_invalid;

    /* Copy I/Q buffer ------------------------------------------------------ */
    pkt.len = (csi_info->len < CSI_BUF_MAX_BYTES)
                  ? csi_info->len
                  : CSI_BUF_MAX_BYTES;
    memcpy(pkt.buf, csi_info->buf, pkt.len);

    /* Push to queue — drop silently if full (Wi-Fi task must not block) ---- */
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_csi_queue, &pkt, &woken) == pdTRUE) {
        s_pkt_count++;
    }
    portYIELD_FROM_ISR(woken);
}

/* ==========================================================================
 * CSI processing task
 *
 * Runs on Core 1 at low priority.  Pops csi_packet_t structs from the queue,
 * builds a CSV text line, and writes it to stdout (→ UART0 → USB cable).
 * ==========================================================================*/
static void csi_process_task(void *arg)
{
    /*  Maximum line length calculation:
     *    Header fields:     "CSI_DATA,1234567890123,-100,-100,14,2,1,1,1,128,"  ≈ 60 chars
     *    128 int8 values:   each up to "-128," = 5 chars  → 128 × 5 = 640 chars
     *    Newline + margin:  10 chars
     *    Total:             ≈ 710 chars.  Use 1024 for safety.
     */
    static char line[1024];
    csi_packet_t pkt;

    while (1) {
        /* Block until a packet is available --------------------------------- */
        if (xQueueReceive(s_csi_queue, &pkt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Determine the valid byte slice ------------------------------------ */
        uint16_t start = pkt.first_word_invalid ? 4u : 0u;
        if (pkt.len <= start) {
            continue;   /* Degenerate packet; skip */
        }
        uint16_t valid_bytes = pkt.len - start;
        int      num_pairs   = (int)(valid_bytes / 2);

        /* Build header ------------------------------------------------------ */
        int pos = snprintf(line, sizeof(line),
                           CSI_DATA_PREFIX
                           ",%lld"   /* timestamp_us      */
                           ",%d"     /* rssi              */
                           ",%d"     /* noise_floor       */
                           ",%u"     /* channel           */
                           ",%u"     /* secondary_channel */
                           ",%u"     /* bandwidth         */
                           ",%u"     /* sig_mode          */
                           ",%u"     /* antenna           */
                           ",%d",    /* num_pairs         */
                           (long long)pkt.timestamp_us,
                           (int)pkt.rssi,
                           (int)pkt.noise_floor,
                           (unsigned)pkt.channel,
                           (unsigned)pkt.secondary_channel,
                           (unsigned)pkt.bandwidth,
                           (unsigned)pkt.sig_mode,
                           (unsigned)pkt.antenna,
                           num_pairs);

        /* Append I/Q values using the fast helper --------------------------- */
        for (uint16_t i = 0; i < valid_bytes; i++) {
            if (pos >= (int)(sizeof(line) - 8)) {
                break;  /* Guard against buffer overflow */
            }
            line[pos++] = ',';
            pos += int8_to_str(line + pos, pkt.buf[start + i]);
        }

        /* Terminate line ---------------------------------------------------- */
        line[pos++] = '\n';
        line[pos]   = '\0';

        /* Write to UART0 (stdout) in one shot for atomicity ----------------- */
        fwrite(line, 1, (size_t)pos, stdout);
        fflush(stdout);
    }
}

/* ==========================================================================
 * Stats task — prints packet rate every 5 seconds (non-CSI_DATA line)
 * ==========================================================================*/
static void csi_stats_task(void *arg)
{
    uint32_t prev = 0;
    const uint32_t period_s = 5;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(period_s * 1000));
        uint32_t now  = s_pkt_count;
        uint32_t rate = (now - prev) / period_s;
        prev = now;
        printf(STATS_PREFIX ",pkt_rate=%lu/s,total=%lu\n",
               (unsigned long)rate, (unsigned long)now);
        fflush(stdout);
    }
}

/* ==========================================================================
 * Public API
 * ==========================================================================*/

void csi_init(void)
{
    /* Create the queue ------------------------------------------------------ */
    s_csi_queue = xQueueCreate(CSI_QUEUE_SIZE, sizeof(csi_packet_t));
    if (s_csi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CSI queue (out of memory)");
        return;
    }

    /* Processing task on Core 1 (Wi-Fi lives on Core 0) ------------------- */
    BaseType_t ret = xTaskCreatePinnedToCore(
        csi_process_task,
        "csi_proc",
        CSI_PROCESS_TASK_STACK,
        NULL,
        CSI_PROCESS_TASK_PRIO,
        NULL,
        1   /* Core 1 */
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create csi_process_task");
        return;
    }

    /* Stats task — any core, very low priority ----------------------------- */
    xTaskCreate(
        csi_stats_task,
        "csi_stats",
        CSI_STATS_TASK_STACK,
        NULL,
        CSI_STATS_TASK_PRIO,
        NULL
    );

    ESP_LOGI(TAG, "CSI subsystem initialised (queue=%d slots, buf_max=%d B)",
             CSI_QUEUE_SIZE, CSI_BUF_MAX_BYTES);
}

void csi_start(void)
{
    /* CSI hardware configuration ------------------------------------------- */
    wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,   /* Legacy LTF — always present         */
        .htltf_en          = true,   /* HT-LTF — present in 802.11n frames  */
        .stbc_htltf2_en    = true,   /* STBC-HT-LTF — present if STBC used  */
        .ltf_merge_en      = true,   /* Merge L-LTF and HT-LTF in one buf  */
        .channel_filter_en = false,  /* Disable HW smoothing (we filter SW) */
        .manu_scale        = false,  /* Auto-scale I/Q values               */
        .shift             = 0,      /* No manual bit-shift                  */
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI enabled — lltf+htltf, 20 MHz, channel_filter=OFF");
}

void csi_stop(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_csi(false));
    ESP_LOGI(TAG, "CSI disabled");
}

uint32_t csi_get_packet_count(void)
{
    return s_pkt_count;
}
