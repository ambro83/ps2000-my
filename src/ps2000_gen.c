#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/*                     SIGNAL GENERATOR CONSTANTS                             */
/* ========================================================================== */

/*
 * Frequency limits for the built-in generator.
 * Reconstructed from the comiss comparisons against dword_A4C84 and
 * dword_A4C88 in sub_2D3A0 (values 100.0f and -100.0f in the listing,
 * used as a symmetric validity window around the normalized frequency).
 */
#define PS2000_GEN_NORM_FREQ_MAX      100.0
#define PS2000_GEN_NORM_FREQ_MIN      (-100.0)

/*
 * DDS reference clock and phase-accumulator width.
 * Reconstructed from the tuning-word multiplication chain in sub_2A850.
 *
 * FIX #12: Changed from 4 MHz to 200 MHz.
 * From sub_2A850: "mov dword ptr [rdi+32Ah], 0BEBC200h" = 200000000 = 200 MHz.
 */
#define PS2000_GEN_DDS_REF_CLK        200000000.0   /* 200 MHz (0x0BEBC200) */
#define PS2000_GEN_PHASE_ACC_MAX      4294967296.0  /* 2^32 */

/*
 * AWG sample-count ceiling.
 * Reconstructed from `cmp esi, 3FFFFFh` in sub_2D3A0.
 */
#define PS2000_GEN_AWG_LIMIT          0x3FFFFFU

/* Waveform identifiers (from the .rodata name table: "HalfSine", "Sinc",
 * "Gaussian", "Square", "Triangle", "RampUp", "RampDown", "DC") */
#define PS2000_GEN_WAVE_SINE          0U
#define PS2000_GEN_WAVE_SQUARE        1U
#define PS2000_GEN_WAVE_TRIANGLE      2U
#define PS2000_GEN_WAVE_RAMP_UP       3U
#define PS2000_GEN_WAVE_RAMP_DOWN     4U
#define PS2000_GEN_WAVE_HALF_SINE     5U
#define PS2000_GEN_WAVE_DC            6U
#define PS2000_GEN_WAVE_SINC          7U
#define PS2000_GEN_WAVE_GAUSSIAN      8U

/* ========================================================================== */
/*                     INTERNAL GENERATOR HELPERS                             */
/* ========================================================================== */

/**
 * @brief Compute the DDS phase increment for a target frequency.
 *
 * Reconstructed from sub_2D3A0:
 *   cvtsi2sd -> mulsd (2^32 / ref_clk) -> cvttsd2si
 *
 * The result is the per-step phase accumulator delta; wrapping is performed
 * by the hardware at 2^32.
 */
static uint32_t _ps2000_gen_calc_phase_inc(double freq_hz) {
    if (freq_hz <= 0.0) return 0U;

    double ratio = freq_hz / PS2000_GEN_DDS_REF_CLK;
    double acc = ratio * PS2000_GEN_PHASE_ACC_MAX;

    if (acc > 4294967295.0) {
        acc = 4294967295.0;
    }

    return (uint32_t)(int64_t)acc;
}

/**
 * @brief Normalize a frequency into the internal symmetric window.
 *
 * Reconstructed from the comiss pair in sub_2D3A0, which checks the value
 * against +100.0 and -100.0 rather than against an absolute Hz range. The
 * generator therefore works on a normalized frequency ratio.
 */
static bool _ps2000_gen_freq_in_window(double normalized) {
    if (normalized > PS2000_GEN_NORM_FREQ_MAX) return false;
    if (normalized < PS2000_GEN_NORM_FREQ_MIN) return false;
    return true;
}

/**
 * @brief Generate one sample of the requested waveform.
 *
 * Uses the shared sine helper so that the PI constant comes from the same
 * source as the rest of the DSP path (_ps2000_math_sine_sample).
 */
static double _ps2000_gen_wave_value(uint32_t wave_type,
                                     uint32_t index,
                                     uint32_t total) {
    if (total == 0U) return 0.0;

    double phase = (double)index / (double)total;

    switch (wave_type) {
        case PS2000_GEN_WAVE_SINE:
            return _ps2000_math_sine_sample(index, total);

        case PS2000_GEN_WAVE_SQUARE:
            return (phase < 0.5) ? 1.0 : -1.0;

        case PS2000_GEN_WAVE_TRIANGLE:
            if (phase < 0.5) {
                return -1.0 + (4.0 * phase);
            }
            return 3.0 - (4.0 * phase);

        case PS2000_GEN_WAVE_RAMP_UP:
            return -1.0 + (2.0 * phase);

        case PS2000_GEN_WAVE_RAMP_DOWN:
            return 1.0 - (2.0 * phase);

        case PS2000_GEN_WAVE_HALF_SINE:
            if (phase < 0.5) {
                return _ps2000_math_sine_sample(index, total / 2U);
            }
            return 0.0;

        case PS2000_GEN_WAVE_DC:
            return 0.0;

        case PS2000_GEN_WAVE_SINC: {
            /* Unnormalized sinc over the full period, centered at the middle
             * of the buffer. Reconstructed from the "Sinc" name entry. */
            double x = (phase - 0.5) * (2.0 * PS2000_MATH_PI);
            if (x == 0.0) return 1.0;
            return sin(x) / x;
        }

        case PS2000_GEN_WAVE_GAUSSIAN: {
            /* Gaussian pulse centered at the middle of the buffer.
             * Reconstructed from the "Gaussian" name entry. */
            double x = (phase - 0.5) * 6.0;
            return exp(-(x * x));
        }

        default:
            return 0.0;
    }
}

/**
 * @brief Validate AWG geometry before upload.
 *
 * Reconstructed from sub_2D3A0: the sample count is compared against
 * 0x3FFFFF, and the normalized frequency must sit inside the symmetric
 * validity window.
 */
static ps2000_status_t _ps2000_gen_validate_awg(uint32_t num_samples,
                                                double normalized_freq) {
    if (num_samples == 0U || num_samples > PS2000_GEN_AWG_LIMIT) {
        PS2000_LOG_WRN("AWG sample count %u outside permitted range",
                       num_samples);
        return PS2000_ERR_SIGGEN_CFG;
    }

    if (!_ps2000_gen_freq_in_window(normalized_freq)) {
        PS2000_LOG_WRN("Normalized frequency %.6f outside window",
                       normalized_freq);
        return PS2000_ERR_SIGGEN_CFG;
    }

    return PS2000_OK;
}

/* ========================================================================== */
/*                     PUBLIC SIGNAL GENERATOR API                            */
/* ========================================================================== */

/**
 * @brief Configure the built-in waveform generator.
 *
 * FIX #8: Renamed from ps2000_set_sig_gen_built_in to
 * _ps2000_gen_set_builtin_internal to avoid collision with the
 * public API signature in ps2000.h.
 *
 * Computes the DDS phase increment, stores the configuration word, and
 * dispatches through the operations table so that device variants can
 * override the behaviour.
 */
ps2000_status_t _ps2000_gen_set_builtin_internal(ps2000_context_t *ctx,
                                                 double freq_hz,
                                                 uint32_t wave_type,
                                                 double amplitude,
                                                 double offset) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    if (wave_type > PS2000_GEN_WAVE_GAUSSIAN) {
        return PS2000_ERR_SIGGEN_CFG;
    }

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    uint32_t phase_inc = _ps2000_gen_calc_phase_inc(freq_hz);

    ctx->siggen.freq_hz = freq_hz;
    ctx->siggen.phase = 0.0;
    ctx->siggen.config = (phase_inc & 0xFFFF0000U) |
                         ((amplitude > 0.0 ? 1U : 0U) << 8U) |
                         (wave_type & 0xFFU);

    PS2000_LOG_INF("Siggen configured: type=%u freq=%.3f Hz inc=0x%08X "
                   "amp=%.3f off=%.3f",
                   wave_type, freq_hz, phase_inc, amplitude, offset);

    if (ctx->ops.validate_siggen) {
        int valid = ctx->ops.validate_siggen(ctx, phase_inc);
        if (valid == 0) {
            pthread_mutex_unlock(&ctx->sync.mutex);
            PS2000_LOG_ERR("Siggen configuration rejected by device");
            return PS2000_ERR_SIGGEN_CFG;
        }
    }

    pthread_mutex_unlock(&ctx->sync.mutex);
    return PS2000_OK;
}

/**
 * @brief Upload an arbitrary waveform to the AWG.
 *
 * FIX #8: Renamed from ps2000_set_sig_gen_arbitrary to
 * _ps2000_gen_set_arbitrary_internal to avoid collision with the
 * public API signature in ps2000.h.
 *
 * Each normalized sample is converted to a 12-bit DAC code through the
 * shared math routine, then streamed to the device over the bulk endpoint.
 */
ps2000_status_t _ps2000_gen_set_arbitrary_internal(ps2000_context_t *ctx,
                                                   const double *waveform,
                                                   uint32_t num_samples,
                                                   double normalized_freq) {
    if (!ctx || !waveform) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    ps2000_status_t status = _ps2000_gen_validate_awg(num_samples,
                                                      normalized_freq);
    if (status != PS2000_OK) return status;

    status = _ps2000_util_mutex_lock_safe(&ctx->sync.mutex,
                                          PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    uint16_t *dac_codes = (uint16_t*)calloc(num_samples, sizeof(uint16_t));
    if (!dac_codes) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return PS2000_ERR_MEMORY;
    }

    for (uint32_t i = 0; i < num_samples; i++) {
        double clamped = waveform[i];
        if (clamped > 1.0) clamped = 1.0;
        if (clamped < -1.0) clamped = -1.0;

        double code = _ps2000_math_awg_to_dac(clamped);
        dac_codes[i] = (uint16_t)(int32_t)code;
    }

    ctx->siggen.awg_buffer_len = (uint16_t)num_samples;
    ctx->siggen.freq_hz = normalized_freq;

    status = _ps2000_usb_bulk_transfer(ctx,
                                       PS2000_USB_EP_OUT,
                                       (const uint8_t*)dac_codes,
                                       num_samples * sizeof(uint16_t));
    free(dac_codes);

    if (status == PS2000_OK) {
        PS2000_LOG_INF("AWG uploaded: %u samples", num_samples);
    } else {
        PS2000_LOG_ERR("AWG upload failed: %d", (int)status);
        ctx->siggen.awg_buffer_len = 0;
    }

    pthread_mutex_unlock(&ctx->sync.mutex);
    return status;
}

/**
 * @brief Generate a built-in waveform into a caller-supplied buffer.
 *
 * Device-independent helper used for preview and self-test; does not require
 * an open device.
 */
ps2000_status_t ps2000_gen_create_waveform(double *buffer,
                                           uint32_t num_samples,
                                           uint32_t wave_type) {
    if (!buffer || num_samples == 0U) return PS2000_ERR_INVALID_PARAM;
    if (wave_type > PS2000_GEN_WAVE_GAUSSIAN) return PS2000_ERR_SIGGEN_CFG;

    for (uint32_t i = 0; i < num_samples; i++) {
        buffer[i] = _ps2000_gen_wave_value(wave_type, i, num_samples);
    }

    return PS2000_OK;
}

/**
 * @brief Reset the signal generator to its default configuration.
 */
ps2000_status_t ps2000_set_sig_gen_off(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    if (ctx->ops.siggen_reset) {
        ctx->ops.siggen_reset(ctx);
    }

    ctx->siggen.freq_hz = 0.0;
    ctx->siggen.phase = 0.0;
    ctx->siggen.awg_buffer_len = 0;
    ctx->siggen.config = PS2000_SIGGEN_DEFAULT;

    PS2000_LOG_INF("Signal generator disabled");
    pthread_mutex_unlock(&ctx->sync.mutex);
    return PS2000_OK;
}