/** @file telemetry.c
 *
 * @brief Pack the shared words into the 21-byte telemetry record.
 *        Signed fields are copied as their two's-complement bit patterns
 *        (same widths on both sides), so no sign handling is needed here.
 */

#include "subsystem1.h"
#include "telemetry.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define BITS_PER_BYTE   8U

_Static_assert(TLM_TOTAL_BITS == (TELEMETRY_BYTES * BITS_PER_BYTE),
               "telemetry layout fills the record exactly");

void
tlm_put (uint8_t * p_buf, uint32_t offset, uint32_t width, uint32_t value)
{
    uint32_t bit;

    for (bit = 0U; bit < width; bit++)
    {
        uint32_t const pos = offset + bit;
        uint32_t const mask = 1UL << (pos % BITS_PER_BYTE);
        uint32_t       byte = p_buf[pos / BITS_PER_BYTE];

        if (0U != ((value >> bit) & 1U))
        {
            byte |= mask;
        }
        else
        {
            byte &= ~mask;
        }
        p_buf[pos / BITS_PER_BYTE] = (uint8_t)byte;
    }
}

uint32_t
tlm_get (uint8_t const * p_buf, uint32_t offset, uint32_t width)
{
    uint32_t bit;
    uint32_t value = 0U;

    for (bit = 0U; bit < width; bit++)
    {
        uint32_t const pos = offset + bit;

        if (0U != (((uint32_t)p_buf[pos / BITS_PER_BYTE] >>
                    (pos % BITS_PER_BYTE)) & 1U))
        {
            value |= 1UL << bit;
        }
    }
    return (value);
}

void
telemetry_pack (telemetry_snapshot_t const * p_snap, uint8_t * p_out)
{
    uint32_t idx;

    for (idx = 0U; idx < TELEMETRY_BYTES; idx++)
    {
        p_out[idx] = 0U;
    }

    tlm_put(p_out, TLM_VERSION_OFS, TLM_VERSION_BITS, TELEMETRY_VERSION);
    tlm_put(p_out, TLM_SEQ_OFS, TLM_SEQ_BITS, p_snap->seq);
    tlm_put(p_out, TLM_MISSION_OFS, TLM_MISSION_BITS,
            bits_get(p_snap->mission, MISSION_STATE_SHIFT,
                     MISSION_STATE_BITS));
    tlm_put(p_out, TLM_MOTION_OFS, TLM_MOTION_BITS,
            bits_get(p_snap->motion, MOTION_STATE_SHIFT, MOTION_STATE_BITS));
    tlm_put(p_out, TLM_SPEED_L_OFS, TLM_SPEED_BITS,
            bits_get(p_snap->motion, MOTION_SPEED_L_SHIFT,
                     MOTION_SPEED_BITS));
    tlm_put(p_out, TLM_SPEED_R_OFS, TLM_SPEED_BITS,
            bits_get(p_snap->motion, MOTION_SPEED_R_SHIFT,
                     MOTION_SPEED_BITS));
    tlm_put(p_out, TLM_ENC_L_OFS, TLM_ENC_BITS, p_snap->enc_left);
    tlm_put(p_out, TLM_ENC_R_OFS, TLM_ENC_BITS, p_snap->enc_right);
    tlm_put(p_out, TLM_LSTATE_OFS, TLM_LSTATE_BITS,
            bits_get(p_snap->line, LINE_STATE_SHIFT, LINE_STATE_BITS));
    tlm_put(p_out, TLM_LERR_OFS, TLM_LERR_BITS,
            bits_get(p_snap->line, LINE_ERR_SHIFT, LINE_ERR_BITS));
    tlm_put(p_out, TLM_BARCODE_OFS, TLM_BARCODE_BITS,
            bits_get(p_snap->line, LINE_LASTBC_SHIFT, LINE_LASTBC_BITS));
    tlm_put(p_out, TLM_REJECT_OFS, TLM_REJECT_BITS,
            bits_get(p_snap->line, LINE_REJECT_SHIFT, LINE_REJECT_BITS));
    tlm_put(p_out, TLM_HUMP_OFS, TLM_HUMP_BITS,
            bits_get(p_snap->hump, 0U, HUMP_PEAK_BITS));
    tlm_put(p_out, TLM_WIDTH_OFS, TLM_WIDTH_BITS,
            bits_get(p_snap->profile0, PROF_WIDTH_SHIFT, PROF_WIDTH_BITS));
    tlm_put(p_out, TLM_CLOSEST_OFS, TLM_CLOSEST_BITS,
            bits_get(p_snap->profile0, PROF_CLOSEST_SHIFT,
                     PROF_CLOSEST_BITS));
    tlm_put(p_out, TLM_BEARING_OFS, TLM_BEARING_BITS,
            bits_get(p_snap->profile0, PROF_BEARING_SHIFT,
                     PROF_BEARING_BITS));
    tlm_put(p_out, TLM_PLAN_OFS, TLM_PLAN_BITS,
            bits_get(p_snap->profile0, PROF_PLAN_SHIFT, PROF_PLAN_BITS));
    tlm_put(p_out, TLM_DIST_OFS, TLM_DIST_BITS,
            bits_get(p_snap->odom, ODOM_TOTAL_SHIFT, ODOM_TOTAL_BITS));
    tlm_put(p_out, TLM_MCLASS_OFS, TLM_MCLASS_BITS,
            bits_get(p_snap->terrain, TERRAIN_CLASS_SHIFT,
                     TERRAIN_CLASS_BITS));
    tlm_put(p_out, TLM_RATE_OFS, TLM_RATE_BITS,
            bits_get(p_snap->terrain, TERRAIN_RATE_SHIFT,
                     TERRAIN_RATE_BITS));
    tlm_put(p_out, TLM_FAULTS_OFS, TLM_FAULTS_BITS, p_snap->faults);
    tlm_put(p_out, TLM_SCAN_OFS, TLM_SCAN_BITS,
            bits_get(p_snap->profile0, PROF_SCAN_SHIFT, PROF_SCAN_BITS));
    tlm_put(p_out, TLM_PITCH_OFS, TLM_PITCH_BITS,
            bits_get(p_snap->terrain, TERRAIN_PITCH_SHIFT,
                     TERRAIN_PITCH_BITS));
    tlm_put(p_out, TLM_ROLL_OFS, TLM_ROLL_BITS,
            bits_get(p_snap->tilt, TILT_ROLL_SHIFT, TILT_ROLL_BITS));
    tlm_put(p_out, TLM_HNOW_OFS, TLM_HNOW_BITS,
            bits_get(p_snap->tilt, TILT_HNOW_SHIFT, TILT_HNOW_BITS));
}

/*** end of file ***/
