/** @file telemetry.h
 *
 * @brief 21-byte binary telemetry record (docs/MQTT.md).  Explicit
 *        shifts and masks into a byte array, LSB-first, never a C struct
 *        on the wire (bit-field layout is implementation-defined and the
 *        dashboard is Python: tools/decode_telemetry.py).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/* Bit offsets and widths in the record (version 1, 168 bits). */
#define TLM_VERSION_OFS     0U
#define TLM_VERSION_BITS    4U
#define TLM_SEQ_OFS         4U
#define TLM_SEQ_BITS        8U
#define TLM_MISSION_OFS     12U
#define TLM_MISSION_BITS    4U
#define TLM_MOTION_OFS      16U
#define TLM_MOTION_BITS     3U
#define TLM_SPEED_L_OFS     19U
#define TLM_SPEED_R_OFS     29U
#define TLM_SPEED_BITS      10U
#define TLM_ENC_L_OFS       39U
#define TLM_ENC_R_OFS       55U
#define TLM_ENC_BITS        16U
#define TLM_LSTATE_OFS      71U
#define TLM_LSTATE_BITS     2U
#define TLM_LERR_OFS        73U
#define TLM_LERR_BITS       8U
#define TLM_BARCODE_OFS     81U
#define TLM_BARCODE_BITS    3U
#define TLM_REJECT_OFS      84U
#define TLM_REJECT_BITS     3U
#define TLM_HUMP_OFS        87U
#define TLM_HUMP_BITS       7U
#define TLM_WIDTH_OFS       94U
#define TLM_WIDTH_BITS      10U
#define TLM_CLOSEST_OFS     104U
#define TLM_CLOSEST_BITS    9U
#define TLM_BEARING_OFS     113U
#define TLM_BEARING_BITS    5U
#define TLM_PLAN_OFS        118U
#define TLM_PLAN_BITS       2U
#define TLM_DIST_OFS        120U
#define TLM_DIST_BITS       15U
#define TLM_MCLASS_OFS      135U
#define TLM_MCLASS_BITS     3U
#define TLM_RATE_OFS        138U
#define TLM_RATE_BITS       10U
#define TLM_FAULTS_OFS      148U
#define TLM_FAULTS_BITS     16U
#define TLM_SCAN_OFS        164U
#define TLM_SCAN_BITS       4U
#define TLM_PITCH_OFS       168U    /* v2: 0.1 deg, signed (+ = nose up) */
#define TLM_PITCH_BITS      10U
#define TLM_ROLL_OFS        178U    /* v2: deg, signed                    */
#define TLM_ROLL_BITS       8U
#define TLM_HNOW_OFS        186U    /* v2: height above flat now, mm      */
#define TLM_HNOW_BITS       6U
#define TLM_TOTAL_BITS      192U

/* One consistent copy of every shared word, taken by T7. */
typedef struct
{
    uint32_t line;
    uint32_t motion;
    uint32_t odom;
    uint32_t terrain;
    uint32_t profile0;
    uint32_t profile1;
    uint32_t tilt;          /* g_tilt_word (v2) */
    uint16_t mission;
    uint16_t faults;
    uint16_t enc_left;
    uint16_t enc_right;
    uint8_t  hump;
    uint8_t  seq;
} telemetry_snapshot_t;

/* Write 'width' bits of 'value' at bit 'offset' (LSB-first stream). */
void     tlm_put(uint8_t * p_buf, uint32_t offset, uint32_t width,
                 uint32_t value);
uint32_t tlm_get(uint8_t const * p_buf, uint32_t offset, uint32_t width);

/* Pack a snapshot into p_out[TELEMETRY_BYTES]. */
void     telemetry_pack(telemetry_snapshot_t const * p_snap,
                        uint8_t * p_out);

#endif /* TELEMETRY_H */

/*** end of file ***/
