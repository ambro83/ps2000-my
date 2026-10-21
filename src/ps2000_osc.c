#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ========================================================================== */
/*                     INTERNAL OSC CONSTANTS                                 */
/* ========================================================================== */

/*
 * Maximum total sample count for a capture block.
 * Reconstructed from `cmp r9w, 0FFEh` / `jbe` comparisons in sub_2ADC0.
 */
#define PS2000_OSC_MAX_SAMPLES        0xFFFU

/*
 * Minimum pre-trigger sample count.
 * Reconstructed from `cmovl edx, esi` where esi = 2 in sub_2ADC0.
 */
#define PS2000_OSC_MIN_PRETRIG        2U

/*
 * Alignment bias used in the ring/capture index calculation.
 * Reconstructed from `add edx, 8000h` in sub_10300+0x1CE.
 */
#define PS2000_OSC_ALIGN_BIAS         0x8000

/*
 * Pre-bias for signed division by 256 (round toward zero).
 * Reconstructed from `lea ecx, [rdx+0FFh]` / `cmovs edx, ecx` / `sar edx, 8`
 * in sub_10300+0x29D..0x2A6.
 */
#define PS2000_OSC_DIV256_PREBIAS     0xFF

/* ========================================================================== */
/*                     INTERNAL OSC HELPERS                                   */
/* ========================================================================== */

/**
 * @brief Signed division by 256 with round-toward-zero semantics.
 *
 * Reconstructed from sub_10300+0x296:
 *   movsx edx, var_6C
 *   test edx, edx
 *   lea  ecx, [rdx+0FFh]
 *   cmovs edx, ecx          ; if negative, add 0xFF first
 *   sar  edx, 8             ; arithmetic shift right by 8
 *
 * For positive values this is a plain floor(x/256). For negative values the
 * 0xFF pre-bias converts the floor into a truncation toward zero, matching
 * the original hardware-side index arithmetic exactly.
 */
static int32_t _ps2000_osc_div256_rounded(int32_t value) {
    int32_t adjusted = value;
    if (value < 0) {
        adjusted = value + PS2000_OSC_DIV256_PREBIAS;
    }
    return adjusted >> 8;
}

/**
 * @brief Scale two sample-derived accumulators by a common normalization factor.
 *
 * Reconstructed from sub_10300+0x17F..0x1CE:
 *   movsx edx, word ptr [rbp+r15*2+293h]   ; reference sample
 *   movsd xmm2, cs:qword_A4C38             ; 16384.0
 *   cvtsi2sd xmm0, edx
 *   movsx eax, word ptr [rbp+r15*2+24Bh]   ; secondary sample
 *   lea  edx, [rcx+rax]                    ; accumulator_a = acc + sample_b
 *   add  eax, var_58                       ; accumulator_b = sample_b + carry
 *   cvtsi2sd xmm1, edx
 *   divsd xmm2, xmm0                       ; factor = 16384.0 / ref_sample
 *   cvtsi2sd xmm0, eax
 *   mulsd xmm1, xmm2
 *   mulsd xmm0, xmm2
 *   cvttsd2si edx, xmm1
 *   cvttsd2si eax, xmm0
 *   add  edx, 8000h                        ; alignment bias
 *
 * Note the direction of the division: 16384.0 is the *numerator*, so the
 * factor grows as the reference sample shrinks. This is an inverse-ratio
 * normalization, not a simple gain.
 *
 * @param ref_sample   Reference sample used as divisor (must be non-zero).
 * @param accum_a      First accumulator value.
 * @param accum_b      Second accumulator value.
 * @param out_a        Output for the first scaled index (alignment applied).
 * @param out_b        Output for the second scaled index.
 */
static void _ps2000_osc_scale_pair(int16_t ref_sample,
                                   int32_t accum_a,
                                   int32_t accum_b,
                                   int32_t *out_a,
                                   int32_t *out_b) {
    if (ref_sample == 0) {
        /* Original code would produce inf/nan here; guard explicitly so the
         * reconstruction does not propagate undefined values into the index
         * arithmetic. */
        *out_a = PS2000_OSC_ALIGN_BIAS;
        *out_b = 0;
        return;
    }

    double factor = PS2000_ADC_FULL_SCALE / (double)ref_sample;
    int32_t scaled_a = (int32_t)(int64_t)((double)accum_a * factor);
    int32_t scaled_b = (int32_t)(int64_t)((double)accum_b * factor);

    *out_a = scaled_a + PS2000_OSC_ALIGN_BIAS;
    *out_b = scaled_b;
}

/**
 * @brief Validate pre/post trigger sample counts against buffer limits.
 *
 * Reconstructed from the validation chain in sub_2ADC0:
 *   - pre-trigger must be >= 2
 *   - total must not exceed 0xFFF
 *   - the addition must not overflow
 *   - total must not exceed the device buffer size when known
 */
ps2000_status_t _ps2000_osc_validate_buffer(uint32_t pre_trig,
                                            uint32_t post_trig,
                                            uint32_t max_samples) {
    if (pre_trig < PS2000_OSC_MIN_PRETRIG) {
        PS2000_LOG_DBG("Pre-trigger %u below minimum %u",
                       pre_trig, PS2000_OSC_MIN_PRETRIG);
        return PS2000_ERR_INVALID_PARAM;
    }

    uint32_t total = pre_trig + post_trig;

    /* Overflow guard on the addition itself */
    if (total < pre_trig) {
        return PS2000_ERR_INVALID_PARAM;
    }

    if (total > PS2000_OSC_MAX_SAMPLES) {
        PS2000_LOG_WRN("Capture total %u exceeds hardware maximum %u",
                       total, PS2000_OSC_MAX_SAMPLES);
        return PS2000_ERR_MEMORY;
    }

    if (max_samples > 0U && total > max_samples) {
        PS2000_LOG_DBG("Capture total %u exceeds buffer size %u",
                       total, max_samples);
        return PS2000_ERR_INVALID_PARAM;
    }

    return PS2000_OK;
}

/**
 * @brief Compute the sample interval for a timebase index.
 *
 * Thin wrapper over the shared math routine so that all timebase arithmetic
 * funnels through one implementation.
 */
ps2000_status_t _ps2000_osc_calc_timebase(uint32_t index, double *interval_ns) {
    if (!interval_ns) return PS2000_ERR_INVALID_PARAM;

    *interval_ns = _ps2000_math_timebase_interval(index);

    if (*interval_ns <= 0.0) {
        return PS2000_ERR_INVALID_PARAM;
    }

    return PS2000_OK;
}

/* ========================================================================== */
/*                     PUBLIC OSCILLOSCOPE API                                */
/* ========================================================================== */

/**
 * @brief Query timebase parameters for a given index and sample count.
 *
 * FIX #8: Renamed from ps2000_get_timebase to _ps2000_osc_get_timebase
 * to avoid collision with the public API signature in ps2000.h.
 *
 * Uses the operations table when available so that device variants can
 * override the calculation, and falls back to the shared math routine
 * otherwise.
 */
ps2000_status_t _ps2000_osc_get_timebase(ps2000_context_t *ctx,
                                         uint32_t timebase_index,
                                         uint32_t num_samples,
                                         double *interval_ns,
                                         uint32_t *max_samples) {
    if (!ctx || !interval_ns) return PS2000_ERR_INVALID_PARAM;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    double interval;
    if (ctx->ops.calc_timebase) {
        interval = ctx->ops.calc_timebase(ctx, timebase_index);
    } else {
        interval = _ps2000_math_timebase_interval(timebase_index);
    }

    if (interval <= 0.0) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return PS2000_ERR_INVALID_PARAM;
    }

    *interval_ns = interval;

    if (max_samples) {
        uint32_t hw_limit = ctx->capture.buffer_size;
        if (hw_limit == 0U || hw_limit > PS2000_OSC_MAX_SAMPLES) {
            hw_limit = PS2000_OSC_MAX_SAMPLES;
        }
        *max_samples = (num_samples < hw_limit) ? num_samples : hw_limit;
    }

    ctx->capture.timebase_index = (uint16_t)timebase_index;
    pthread_mutex_unlock(&ctx->sync.mutex);
    return PS2000_OK;
}

/**
 * @brief Configure and start a block capture.
 *
 * FIX #8: Renamed from ps2000_run_block to _ps2000_osc_run_block
 * to avoid collision with the public API signature in ps2000.h.
 *
 * Validates the buffer geometry, allocates the per-channel capture buffers,
 * then dispatches through the operations table.
 */
ps2000_status_t _ps2000_osc_run_block(ps2000_context_t *ctx,
                                      uint32_t pre_trigger_samples,
                                      uint32_t post_trigger_samples) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    ps2000_status_t status = _ps2000_osc_validate_buffer(
        pre_trigger_samples, post_trigger_samples, ctx->capture.buffer_size);
    if (status != PS2000_OK) return status;

    status = _ps2000_util_mutex_lock_safe(&ctx->sync.mutex,
                                          PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    uint32_t total = pre_trigger_samples + post_trigger_samples;

    /* Allocate per-channel buffers if the geometry changed */
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        if ((ctx->capture.channel_mask & (1U << ch)) == 0U) {
            continue;
        }
        if (ctx->capture.channel_buffers[ch] == NULL) {
            ctx->capture.channel_buffers[ch] =
                (int16_t*)calloc(total, sizeof(int16_t));
            if (!ctx->capture.channel_buffers[ch]) {
                pthread_mutex_unlock(&ctx->sync.mutex);
                PS2000_LOG_ERR("Failed to allocate capture buffer for ch %u", ch);
                return PS2000_ERR_MEMORY;
            }
        }
    }

    ctx->capture.pre_trigger = pre_trigger_samples;
    ctx->capture.buffer_size = total;
    ctx->capture.pending_ops = (uint16_t)ctx->capture.channel_mask;

    PS2000_LOG_INF("Block capture armed: pre=%u post=%u mask=0x%02X",
                   pre_trigger_samples, post_trigger_samples,
                   ctx->capture.channel_mask);

    if (ctx->ops.run_block) {
        status = (ps2000_status_t)ctx->ops.run_block(ctx,
                                                     pre_trigger_samples,
                                                     post_trigger_samples);
    } else {
        status = PS2000_ERR_NOT_IMPLEMENTED;
    }

    if (status != PS2000_OK) {
        PS2000_LOG_ERR("Block capture dispatch failed: %d", (int)status);
        ctx->capture.capture_state = 0;
    }

    pthread_mutex_unlock(&ctx->sync.mutex);
    return status;
}

/**
 * @brief Read calibrated samples out of a completed capture.
 *
 * FIX #8: Renamed from ps2000_get_values to _ps2000_osc_get_values
 * to avoid collision with the public API signature in ps2000.h.
 *
 * Copies the raw window and applies the per-channel calibration chain.
 * The conversion itself is delegated to _ps2000_math_adc_convert so the
 * saturation behaviour matches the original `lea edx,[rax+7FFEh]` /
 * `cmp edx, 0FFFCh` clamp exactly.
 */
ps2000_status_t _ps2000_osc_get_values(ps2000_context_t *ctx,
                                       uint32_t channel,
                                       int16_t *buffer,
                                       uint32_t start_index,
                                       uint32_t num_samples,
                                       uint32_t *returned_samples) {
    if (!ctx || !buffer || !returned_samples) return PS2000_ERR_INVALID_PARAM;
    if (channel >= PS2000_MAX_CHANNELS) return PS2000_ERR_INVALID_PARAM;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    int16_t *source = ctx->capture.channel_buffers[channel];
    if (!source) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        PS2000_LOG_ERR("No capture buffer for channel %u", channel);
        return PS2000_ERR_NOT_OPENED;
    }

    if (start_index >= ctx->capture.buffer_size) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        *returned_samples = 0U;
        return PS2000_OK;
    }

    uint32_t available = ctx->capture.buffer_size - start_index;
    uint32_t count = (num_samples < available) ? num_samples : available;

    double gain = ctx->info.cal_gain[channel];
    double offset = (double)ctx->info.cal_offset[channel];

    for (uint32_t i = 0; i < count; i++) {
        int16_t raw = source[start_index + i];
        buffer[i] = _ps2000_math_adc_convert(raw, gain, offset);
    }

    *returned_samples = count;
    PS2000_LOG_DBG("Read %u samples from channel %u at offset %u",
                   count, channel, start_index);

    pthread_mutex_unlock(&ctx->sync.mutex);
    return PS2000_OK;
}

/**
 * @brief Compute decimated capture indices for a sample pair.
 *
 * Exposes the sub_10300 scaling path for internal aggregation use. The two
 * output indices are the aligned primary index and the secondary index used
 * for interpolation between the two capture arrays.
 */
void _ps2000_osc_compute_decim_indices(int16_t ref_sample,
                                       int16_t secondary_sample,
                                       int32_t carry,
                                       int32_t *out_primary,
                                       int32_t *out_secondary,
                                       int32_t *out_div256) {
    if (!out_primary || !out_secondary || !out_div256) return;

    int32_t accum_a = (int32_t)secondary_sample;
    int32_t accum_b = (int32_t)secondary_sample + carry;
    int32_t primary = 0;
    int32_t secondary = 0;

    _ps2000_osc_scale_pair(ref_sample, accum_a, accum_b, &primary, &secondary);

    *out_primary = primary;
    *out_secondary = secondary;
    *out_div256 = _ps2000_osc_div256_rounded(secondary);
}