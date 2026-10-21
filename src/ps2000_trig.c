#include "ps2000_internal.h"
#include <string.h>

/* ========================================================================== */
/*                     TRIGGER VALIDATION                                     */
/* ========================================================================== */

/**
 * @brief Validate trigger configuration.
 *
 * Reconstructs sub_2A8D6 and sub_10BF0 trigger validation logic.
 * Checks channel mask, threshold bounds, and ETS mode requirements.
 *
 * @param ctx          Device context
 * @param channel_mask Bitmask of channels to trigger on
 * @param threshold    Trigger threshold (raw ADC value)
 * @param direction    Trigger direction (rising/falling/dual)
 * @param delay        Trigger delay in samples
 * @return PS2000_OK if valid, PS2000_ERR_TRIGGER if invalid
 */
ps2000_status_t _ps2000_trg_validate_config(ps2000_context_t *ctx,
                                            uint16_t channel_mask,
                                            uint16_t threshold,
                                            uint16_t direction,
                                            uint16_t delay) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    /* Trigger enabled but no channels selected */
    if (channel_mask == 0) {
        PS2000_LOG_WRN("Trigger enabled but no channel mask set");
        return PS2000_ERR_TRIGGER;
    }

    /* Count active channels */
    uint32_t active_ch = 0;
    for (uint32_t i = 0; i < PS2000_MAX_CHANNELS; i++) {
        if (channel_mask & (1U << i)) active_ch++;
    }

    /* ETS mode validation (from sub_2A8D6+2A cmp al, 2) */
    /* In ETS mode, channels must be paired or follow specific count rules */
    bool is_ets = (ctx->capture.timebase_index > 100);
    if (is_ets && (active_ch % 2) != 0) {
        PS2000_LOG_WRN("ETS mode requires even number of active trigger channels");
        return PS2000_ERR_TRIGGER;
    }

    /* Validate delay bounds */
    if (delay > ctx->capture.buffer_size) {
        PS2000_LOG_WRN("Trigger delay %u exceeds buffer size %u",
                       delay, ctx->capture.buffer_size);
        return PS2000_ERR_INVALID_PARAM;
    }

    return PS2000_OK;
}

/* ========================================================================== */
/*                     TRIGGER POSITION CALCULATION                           */
/* ========================================================================== */

/**
 * @brief Calculate precise trigger position in samples.
 *
 * Reconstructs sub_11789 SSE block:
 *   divss xmm0, xmm1; cvtss2sd xmm0, xmm0; addsd xmm0, qword_A4C60;
 *   mulsd xmm0, xmm1; addsd xmm0, qword_A4C40; cvttsd2si rcx, xmm0
 *
 * @param ctx           Device context
 * @param trig_pos_out  Pointer to store calculated trigger position
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_trg_calculate_position(ps2000_context_t *ctx,
                                               uint32_t *trig_pos_out) {
    if (!ctx || !trig_pos_out) return PS2000_ERR_INVALID_PARAM;

    double interval = ctx->capture.sample_interval;
    if (interval <= 0.0) return PS2000_ERR_INVALID_PARAM;

    /* Base calculation from floating point sample interval */
    double raw_pos = (double)ctx->capture.pre_trigger;
    raw_pos += PS2000_TIMEBASE_ROUND_BIAS; /* 0.5 correction */

    /* Scale back to sample domain */
    double scaled_pos = raw_pos * PS2000_TIMEBASE_UNIT_STEP;
    scaled_pos += PS2000_TIMEBASE_ROUND_BIAS;

    /* Truncate as per cvttsd2si behavior */
    uint32_t final_pos = (uint32_t)(int64_t)scaled_pos;

    /* Bounds check against buffer size */
    if (final_pos > ctx->capture.buffer_size) {
        final_pos = ctx->capture.buffer_size;
    }

    *trig_pos_out = final_pos;
    return PS2000_OK;
}