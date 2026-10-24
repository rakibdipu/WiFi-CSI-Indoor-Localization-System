/* ==========================================================================
 * config.h — User-configurable project settings
 *
 * EDIT THIS FILE before flashing:
 *   1. Set WIFI_SSID  to your phone hotspot name
 *   2. Set WIFI_PASSWORD to your phone hotspot password
 *
 * All other values have sensible defaults for the ESP32-S3-CAM N16R8.
 * ==========================================================================*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Wi-Fi Credentials  (your phone hotspot)
 * --------------------------------------------------------------------------*/
#define WIFI_SSID           "YourPhoneHotspotSSID"   /* <-- CHANGE THIS */
#define WIFI_PASSWORD       "YourHotspotPassword"    /* <-- CHANGE THIS */
#define WIFI_MAX_RETRY      10                        /* reconnect attempts */

/* --------------------------------------------------------------------------
 * CSI Module
 * --------------------------------------------------------------------------*/
/* Number of CSI packets held in the FreeRTOS queue between the callback
 * and the processing task.  Each slot is sizeof(csi_packet_t) ≈ 270 bytes.
 * 32 slots × 270 B = ~8.6 KB of internal SRAM. Safe on ESP32-S3.          */
#define CSI_QUEUE_SIZE              32

/* Stack sizes (bytes) for FreeRTOS tasks                                   */
#define CSI_PROCESS_TASK_STACK      8192
#define CSI_STATS_TASK_STACK        2048

/* FreeRTOS task priorities (higher number = higher priority)
 * Wi-Fi task runs at priority 23 (ESP-IDF default). Keep ours lower.      */
#define CSI_PROCESS_TASK_PRIO       5
#define CSI_STATS_TASK_PRIO         2
#define TRAFFIC_GEN_TASK_PRIO       4
#define TRAFFIC_GEN_TASK_STACK      4096

/* --------------------------------------------------------------------------
 * Traffic Generation
 * ESP32 sends small UDP packets to the phone (gateway) so the phone's
 * 802.11 MAC layer sends ACK frames back. Those ACK frames are received by
 * the ESP32 and trigger the CSI callback → steady downlink CSI stream.
 * --------------------------------------------------------------------------*/
#define TRAFFIC_GEN_INTERVAL_MS     10      /* 10 ms → ~100 packets/sec     */
#define TRAFFIC_GEN_DST_PORT        5555    /* Arbitrary port; phone ignores */

/* --------------------------------------------------------------------------
 * Serial Output
 * Lines prefixed with CSI_DATA_PREFIX are parsed by the Python receiver.
 * Other lines (ESP_LOG, STATS lines) are ignored by the parser.
 * --------------------------------------------------------------------------*/
#define CSI_DATA_PREFIX     "CSI_DATA"
#define STATS_PREFIX        "STATS"

/* Max CSI buffer size per packet.
 * HT-LTF + L-LTF at 20 MHz → 256 bytes max.
 * 40 MHz mode would need 512 bytes; we cap here for Phase 1 (20 MHz only). */
#define CSI_BUF_MAX_BYTES   256

/* --------------------------------------------------------------------------
 * LED indicator  (set GPIO to -1 to disable)
 * ESP32-S3-CAM boards vary. Common choices: GPIO2, GPIO48 (WS2812 on some).
 * This is a plain GPIO toggle (not WS2812) — just adjust the GPIO number.  */
#define STATUS_LED_GPIO     2     /* -1 = disabled                          */

#ifdef __cplusplus
}
#endif
