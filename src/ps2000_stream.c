#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>

/* ========================================================================== */
/*                     STREAMING MATH & RING BUFFER                           */
/* ========================================================================== */

/**
 * @brief Update ring buffer write position with wrap-around.
 *
 * FIX #8: Reconstructs sub_10300 / sub_10770 with correct logic:
 *   add eax, 0FFh          ; bias for rounding up
 *   and eax, 0FFFFFF00h    ; align DOWN to 256-byte boundary
 *   and eax, (buf_size-1)  ; wrap using power-of-2 mask
 *
 * The buffer size MUST be a power of 2 for the mask-based wrap to work.
 * This is enforced by the streaming setup code (sub_2B6FD).
 *
 * Implements aligned circular buffer indexing.
 */
uint32_t _ps2000_strm_update_ring_pos(uint32_t current_pos, uint32_t increment,
                                      uint32_t buf_size) {
    if (buf_size == 0) return 0;
    uint32_t new_pos = current_pos + increment;
    /* FIX #8: Round UP to next 256-byte boundary, then wrap.
     * Original asm: add 0FFh; and 0FFFFFF00h; and (buf_size-1) */
    uint32_t aligned = (new_pos + 0xFFU) & 0xFFFFFF00U;
    /* Wrap around using power-of-2 mask (buf_size must be power of 2) */
    return aligned & (buf_size - 1U);
}

/**
 * @brief Process raw streaming samples with calibration and correction.
 *
 * Reconstructs sub_28A19 SSE block:
 *   cvtsi2sd xmm0, [mem]; divsd xmm0, 16384.0; mulsd xmm0, gain;
 *   addsd xmm0, correction; mulsd xmm0, 32768.0; cvttsd2si esi, xmm0;
 *
 * FIX #2: Removed double division of offset.
 */
ps2000_status_t _ps2000_strm_process_raw(ps2000_context_t *ctx,
                                         const int16_t *raw_buf,
                                         uint32_t num_samples,
                                         int16_t *out_buf,
                                         uint32_t *processed_count) {
    if (!ctx || !raw_buf || !out_buf || !processed_count) {
        return PS2000_ERR_INVALID_PARAM;
    }
    if (num_samples == 0) {
        *processed_count = 0;
        return PS2000_OK;
    }

    /* Use channel 0 calibration by default for streaming */
    double gain = ctx->info.cal_gain[0];
    /* FIX #2: Use offset directly without dividing by 256.0 */
    double offset = (double)ctx->info.cal_offset[0];

    for (uint32_t i = 0; i < num_samples; i++) {
        /* Delegate to centralized math module with correct formula */
        out_buf[i] = _ps2000_math_adc_convert(raw_buf[i], gain, offset);
    }

    *processed_count = num_samples;
    return PS2000_OK;
}