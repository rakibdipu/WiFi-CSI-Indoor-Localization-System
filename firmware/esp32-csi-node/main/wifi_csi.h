/* ==========================================================================
 * wifi_csi.h — CSI subsystem public interface
 * ==========================================================================*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * csi_packet_t
 *
 * A single CSI measurement snapshot.  One of these is produced per received
 * Wi-Fi frame and passed through the FreeRTOS queue from the CSI callback
 * to the processing task.
 *
 * Memory layout of buf[] (for 20 MHz HT mode, ltf_merge_en = true):
 *   [0 .. 103]  → L-LTF:  52 subcarriers × (int8 I, int8 Q)
 *   [104 .. 215]→ HT-LTF: 56 subcarriers × (int8 I, int8 Q)
 *   Subcarrier pairs stored in hardware order; Python receiver re-sorts them
 *   into frequency-monotonic order using the ESP-IDF subcarrier index table.
 *
 * NOTE: If first_word_invalid == true, skip the first 4 bytes before
 *       interpreting the buffer contents.
 * --------------------------------------------------------------------------*/
typedef struct {
    int64_t  timestamp_us;      /**< esp_timer_get_time() at callback entry  */
    int8_t   rssi;              /**< Received Signal Strength (dBm)          */
    int8_t   noise_floor;       /**< RF noise floor estimate (dBm)           */
    uint8_t  channel;           /**< Primary Wi-Fi channel (1–14)            */
    uint8_t  secondary_channel; /**< 0=none, 1=above, 2=below                */
    uint8_t  sig_mode;          /**< 0=Non-HT, 1=HT (11n), 3=VHT (11ac)    */
    uint8_t  bandwidth;         /**< 0=20 MHz, 1=40 MHz                      */
    uint8_t  antenna;           /**< Antenna index (0 or 1)                  */
    bool     first_word_invalid;/**< If true, skip first 4 bytes of buf      */
    uint16_t len;               /**< Valid bytes in buf (= 2 × N_subcarriers)*/
    int8_t   buf[CSI_BUF_MAX_BYTES]; /**< Raw I/Q buffer                    */
} csi_packet_t;

/* --------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

/**
 * @brief  Create the internal queue and spawn the processing task.
 *         Call once from app_main(), before csi_start().
 */
void csi_init(void);

/**
 * @brief  Register the CSI callback, apply CSI configuration, and enable CSI.
 *         Must be called AFTER the station has connected (IP_EVENT_STA_GOT_IP).
 */
void csi_start(void);

/**
 * @brief  Disable CSI capture.
 */
void csi_stop(void);

/**
 * @brief  Return the total number of CSI packets received since boot.
 */
uint32_t csi_get_packet_count(void);

#ifdef __cplusplus
}
#endif
