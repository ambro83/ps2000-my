#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/*                     SIGNAL GENERATOR CONSTANTS                             */
/* ========================================================================== */
/*
 * Frequency limits for the built-in generator.
 * From sub_2D3A0: comiss against dword_A4C84 (100.0f) and dword_A4C88 (-100.0f)
 */
#define PS2000_GEN_NORM_FREQ_MAX      100.0
#define PS2000_GEN_NORM_FREQ_MIN      (-100.0)

/*
 * DDS reference clock and phase-accumulator width.
 * From sub_2A850: "mov dword ptr [rdi+32Ah], 0BEBC200h" = 200 MHz
 * From sub_35E40: phase_inc = freq / 200MHz * 2^32
 */
#define PS2000_GEN_DDS_REF_CLK        200000000.0   /* 200 MHz */
#define PS2000_GEN_PHASE_ACC_MAX      4294967296.0  /* 2^32 */

/*
 * AWG sample-count ceiling.
 * From sub_2D3A0: cmp esi, 3FFFFFh
 */
#define PS2000_GEN_AWG_LIMIT          0x3FFFFFU

/*
 * ========================================================================
 * FIX #200: Wave type identifiers — MUST match ps2000.h PS2000_WAVE_TYPE
 * ========================================================================
 *
 * Original (WRONG):
 *   HALF_SINE=5, DC=6, SINC=7, GAUSSIAN=8
 *
 * Correct (matches ps2000.h enum enPS2000WaveType):
 *   SINE=0, SQUARE=1, TRIANGLE=2, RAMPUP=3, RAMPDOWN=4,
 *   DC_VOLTAGE=5, GAUSSIAN=6, SINC=7, HALF_SINE=8
 */
#define PS2000_GEN_WAVE_SINE          0U   /* PS2000_SINE */
#define PS2000_GEN_WAVE_SQUARE        1U   /* PS2000_SQUARE */
#define PS2000_GEN_WAVE_TRIANGLE      2U   /* PS2000_TRIANGLE */
#define PS2000_GEN_WAVE_RAMP_UP       3U   /* PS2000_RAMPUP */
#define PS2000_GEN_WAVE_RAMP_DOWN     4U   /* PS2000_RAMPDOWN */
#define PS2000_GEN_WAVE_DC_VOLTAGE    5U   /* PS2000_DC_VOLTAGE — FIX #200 */
#define PS2000_GEN_WAVE_GAUSSIAN      6U   /* PS2000_GAUSSIAN — FIX #200 */
#define PS2000_GEN_WAVE_SINC          7U   /* PS2000_SINC */
#define PS2000_GEN_WAVE_HALF_SINE     8U   /* PS2000_HALF_SINE — FIX #200 */

/* ========================================================================== */
/*                     SWEEP STATE                                            */
/* ========================================================================== */
/*
 * FIX #203: Sweep state for frequency sweep generation.
 * Reconstructed from sub_2D3A0 and sub_35E40 patterns.
 *
 * The DDS hardware sweeps by incrementing/decrementing the phase accumulator
 * delta (deltaPhase) at a rate determined by dwellCount.
 */
typedef struct _ps2000_gen_sweep_state {
    double      start_freq;         /* Starting frequency (Hz) */
    double      stop_freq;          /* Ending frequency (Hz) */
    double      freq_increment;     /* Frequency step (Hz) */
    double      dwell_time;         /* Time at each step (seconds) */
    uint32_t    sweep_type;         /* PS2000_SWEEP_TYPE */
    uint32_t    sweeps;             /* Number of complete sweeps (0 = infinite) */
    /* Runtime state */
    double      current_freq;       /* Current frequency */
    int32_t     sweep_direction;    /* +1 = up, -1 = down */
    uint32_t    completed_sweeps;   /* Count of completed sweeps */
    double      dwell_accumulator;  /* Time accumulator for dwell */
    bool        active;             /* Sweep is running */
} ps2000_gen_sweep_state_t;

static ps2000_gen_sweep_state_t g_sweep_state = {0};

/* ========================================================================== */
/*                     INTERNAL GENERATOR HELPERS                             */
/* ========================================================================== */

/**
 * @brief Compute the DDS phase increment for a target frequency.
 *
 * Reconstructed from sub_35E40:
 *   cvtsi2sd xmm0, freq
 *   divsd    xmm0, cs:qword_200MHz    ; 200000000.0
 *   mulsd    xmm0, cs:qword_2^32      ; 4294967296.0
 *   cvttsd2si eax, xmm0
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
 * From sub_2D3A0: comiss against +100.0f and -100.0f
 */
static bool _ps2000_gen_freq_in_window(double normalized) {
    if (normalized > PS2000_GEN_NORM_FREQ_MAX) return false;
    if (normalized < PS2000_GEN_NORM_FREQ_MIN) return false;
    return true;
}

/**
 * @brief Generate one sample of the requested waveform.
 *
 * FIX #200: Corrected wave type numbering to match ps2000.h
 * FIX #201: Corrected HALF_SINE and DC_VOLTAGE implementations
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

        /*
         * FIX #201: DC voltage generates a constant waveform.
         * The actual DC level is determined by offsetVoltage/pkToPk
         * parameters passed to ps2000_set_sig_gen_built_in().
         * The waveform buffer is filled with a constant value.
         */
        case PS2000_GEN_WAVE_DC_VOLTAGE:
            return 1.0;  /* Constant — amplitude scaling applied externally */

        /*
         * FIX #200: Gaussian uses enum value 6 (was 8)
         * Gaussian pulse centered at the middle of the buffer.
         * sigma = 1/6 of the buffer width (from original constant 6.0)
         */
        case PS2000_GEN_WAVE_GAUSSIAN: {
            double x = (phase - 0.5) * 6.0;
            return exp(-(x * x));
        }

        /*
         * FIX #201: Sinc — normalized sinc function
         * sinc(x) = sin(pi*x) / (pi*x)
         * Centered at the middle of the buffer, spanning [-1, +1]
         */
        case PS2000_GEN_WAVE_SINC: {
            double x = (phase - 0.5) * 2.0;  /* x in [-1, +1] */
            double pix = PS2000_MATH_PI * x;
            if (fabs(pix) < 1e-10) return 1.0;  /* sinc(0) = 1 */
            return sin(pix) / pix;
        }

        /*
         * FIX #200 + #201: Half sine uses enum value 8 (was 5)
         * Half sine: sin(pi * phase) for the full period [0, 1]
         * This creates a single positive half-cycle of a sine wave.
         */
        case PS2000_GEN_WAVE_HALF_SINE:
            return sin(PS2000_MATH_PI * phase);

        default:
            return 0.0;
    }
}

/**
 * @brief Validate AWG geometry before upload.
 *
 * From sub_2D3A0: cmp esi, 3FFFFFh + comiss range check
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
/*                     SWEEP IMPLEMENTATION                                   */
/* ========================================================================== */
/*
 * FIX #203: Sweep state machine.
 *
 * From sub_2D3A0 analysis:
 * - startDeltaPhase / stopDeltaPhase define the frequency range
 * - deltaPhaseIncrement defines the step size
 * - dwellCount defines how many DDS cycles at each frequency
 * - sweepType: UP, DOWN, UPDOWN, DOWNUP
 * - sweeps: number of complete sweeps (0 = infinite)
 *
 * The sweep is implemented by a timer-driven state machine that
 * updates the phase accumulator delta at each dwell interval.
 */
static void _ps2000_gen_sweep_init(double start_freq,
                                   double stop_freq,
                                   double increment,
                                   double dwell_time,
                                   uint32_t sweep_type,
                                   uint32_t sweeps) {
    memset(&g_sweep_state, 0, sizeof(g_sweep_state));
    g_sweep_state.start_freq = start_freq;
    g_sweep_state.stop_freq = stop_freq;
    g_sweep_state.freq_increment = increment;
    g_sweep_state.dwell_time = dwell_time;
    g_sweep_state.sweep_type = sweep_type;
    g_sweep_state.sweeps = sweeps;

    /* Initialize based on sweep direction */
    switch (sweep_type) {
        case 0: /* PS2000_UP */
            g_sweep_state.current_freq = start_freq;
            g_sweep_state.sweep_direction = +1;
            break;
        case 1: /* PS2000_DOWN */
            g_sweep_state.current_freq = stop_freq;
            g_sweep_state.sweep_direction = -1;
            break;
        case 2: /* PS2000_UPDOWN */
            g_sweep_state.current_freq = start_freq;
            g_sweep_state.sweep_direction = +1;
            break;
        case 3: /* PS2000_DOWNUP */
            g_sweep_state.current_freq = stop_freq;
            g_sweep_state.sweep_direction = -1;
            break;
        default:
            g_sweep_state.current_freq = start_freq;
            g_sweep_state.sweep_direction = +1;
            break;
    }
    g_sweep_state.completed_sweeps = 0;
    g_sweep_state.dwell_accumulator = 0.0;
    g_sweep_state.active = true;
}

/**
 * @brief Advance the sweep state by one time step.
 * @return true if the frequency was updated, false if still dwelling
 */
static bool _ps2000_gen_sweep_step(double dt, double *new_freq_out) {
    if (!g_sweep_state.active) return false;

    g_sweep_state.dwell_accumulator += dt;
    if (g_sweep_state.dwell_accumulator < g_sweep_state.dwell_time) {
        /* Still dwelling at current frequency */
        *new_freq_out = g_sweep_state.current_freq;
        return false;
    }

    /* Dwell time exceeded — advance frequency */
    g_sweep_state.dwell_accumulator -= g_sweep_state.dwell_time;
    double next_freq = g_sweep_state.current_freq +
                       g_sweep_state.sweep_direction * g_sweep_state.freq_increment;

    /* Check if we've reached the end of the sweep range */
    bool reached_end = false;
    if (g_sweep_state.sweep_direction > 0) {
        /* Sweeping up */
        if (next_freq > g_sweep_state.stop_freq) {
            reached_end = true;
        }
    } else {
        /* Sweeping down */
        if (next_freq < g_sweep_state.stop_freq) {
            reached_end = true;
        }
    }

    if (reached_end) {
        g_sweep_state.completed_sweeps++;
        /* Check if we've done all requested sweeps */
        if (g_sweep_state.sweeps > 0 &&
            g_sweep_state.completed_sweeps >= g_sweep_state.sweeps) {
            g_sweep_state.active = false;
            *new_freq_out = g_sweep_state.current_freq;
            return true;
        }
        /* Handle sweep type for direction change */
        switch (g_sweep_state.sweep_type) {
            case 0: /* UP — restart from start */
                g_sweep_state.current_freq = g_sweep_state.start_freq;
                break;
            case 1: /* DOWN — restart from stop */
                g_sweep_state.current_freq = g_sweep_state.stop_freq;
                break;
            case 2: /* UPDOWN — reverse direction */
                g_sweep_state.sweep_direction = -1;
                g_sweep_state.current_freq = g_sweep_state.stop_freq;
                break;
            case 3: /* DOWNUP — reverse direction */
                g_sweep_state.sweep_direction = +1;
                g_sweep_state.current_freq = g_sweep_state.start_freq;
                break;
        }
    } else {
        g_sweep_state.current_freq = next_freq;
    }

    *new_freq_out = g_sweep_state.current_freq;
    return true;
}

/* ========================================================================== */
/*                     PUBLIC SIGNAL GENERATOR API                            */
/* ========================================================================== */

/**
 * @brief Configure the built-in waveform generator.
 *
 * FIX #203: Added sweep parameters (startFrequency, stopFrequency, etc.)
 *
 * Reconstructs ps2000_set_sig_gen_built_in from the public API.
 * The original function signature from ps2000.h:
 *   ps2000_set_sig_gen_built_in(handle, offsetVoltage, pkToPk,
 *     waveType, startFrequency, stopFrequency, increment, dwellTime,
 *     sweepType, sweeps)
 */
ps2000_status_t _ps2000_gen_set_builtin_internal(ps2000_context_t *ctx,
                                                 double offset_voltage,
                                                 double pk_to_pk,
                                                 uint32_t wave_type,
                                                 double start_frequency,
                                                 double stop_frequency,
                                                 double increment,
                                                 double dwell_time,
                                                 uint32_t sweep_type,
                                                 uint32_t sweeps) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    /* FIX #200: Validate wave type against corrected enum */
    if (wave_type > PS2000_GEN_WAVE_HALF_SINE) {
        return PS2000_ERR_SIGGEN_CFG;
    }

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    /* Calculate phase increment for start frequency */
    uint32_t start_phase_inc = _ps2000_gen_calc_phase_inc(start_frequency);

    /* Store configuration */
    ctx->siggen.freq_hz = start_frequency;
    ctx->siggen.phase = 0.0;
    ctx->siggen.config = (start_phase_inc & 0xFFFF0000U) |
                         ((pk_to_pk > 0.0 ? 1U : 0U) << 8U) |
                         (wave_type & 0xFFU);

    /*
     * FIX #203: Initialize sweep if start != stop frequency
     *
     * From sub_2D3A0: if startDeltaPhase != stopDeltaPhase,
     * then sweep mode is activated.
     */
    bool is_sweep = (fabs(start_frequency - stop_frequency) > 1e-6) &&
                    (sweep_type < 4);  /* MAX_SWEEP_TYPES = 4 */

    if (is_sweep && increment > 0.0 && dwell_time > 0.0) {
        _ps2000_gen_sweep_init(start_frequency, stop_frequency,
                               increment, dwell_time,
                               sweep_type, sweeps);
        PS2000_LOG_INF("Siggen sweep configured: type=%u freq=%.1f→%.1f Hz "
                       "inc=%.3f dwell=%.3f sweeps=%u",
                       sweep_type, start_frequency, stop_frequency,
                       increment, dwell_time, sweeps);
    } else {
        /* Single frequency mode — disable sweep */
        g_sweep_state.active = false;
        PS2000_LOG_INF("Siggen configured: type=%u freq=%.3f Hz "
                       "offset=%.3f pkpk=%.3f",
                       wave_type, start_frequency, offset_voltage, pk_to_pk);
    }

    /* Validate through ops table */
    if (ctx->ops.validate_siggen) {
        int valid = ctx->ops.validate_siggen(ctx, start_phase_inc);
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
 * Reconstructs ps2000_set_sig_gen_arbitrary.
 * Each normalized sample is converted to a 12-bit DAC code through
 * _ps2000_math_awg_to_dac, then streamed over the bulk endpoint.
 *
 * The original function signature from ps2000.h:
 *   ps2000_set_sig_gen_arbitrary(handle, offsetVoltage, pkToPk,
 *     startDeltaPhase, stopDeltaPhase, deltaPhaseIncrement, dwellCount,
 *     arbitraryWaveform, arbitraryWaveformSize, sweepType, sweeps)
 */
ps2000_status_t _ps2000_gen_set_arbitrary_internal(ps2000_context_t *ctx,
                                                   int32_t offset_voltage,
                                                   uint32_t pk_to_pk,
                                                   uint32_t start_delta_phase,
                                                   uint32_t stop_delta_phase,
                                                   uint32_t delta_phase_increment,
                                                   uint32_t dwell_count,
                                                   const uint8_t *waveform,
                                                   int32_t waveform_size,
                                                   uint32_t sweep_type,
                                                   uint32_t sweeps) {
    if (!ctx || !waveform) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    /* Validate waveform size */
    if (waveform_size <= 0 || (uint32_t)waveform_size > PS2000_GEN_AWG_LIMIT) {
        PS2000_LOG_WRN("AWG waveform size %d outside permitted range",
                       waveform_size);
        return PS2000_ERR_SIGGEN_CFG;
    }

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return status;

    /*
     * FIX #203: Configure sweep for arbitrary waveform.
     *
     * The arbitrary waveform sweep uses deltaPhase values directly
     * (not frequencies). The hardware increments/decrements the
     * phase accumulator delta at each dwell_count interval.
     */
    if (start_delta_phase != stop_delta_phase &&
        delta_phase_increment > 0 && dwell_count > 0) {
        /* Convert deltaPhase to frequency for sweep state */
        double start_freq = (double)start_delta_phase / PS2000_GEN_PHASE_ACC_MAX
                            * PS2000_GEN_DDS_REF_CLK;
        double stop_freq = (double)stop_delta_phase / PS2000_GEN_PHASE_ACC_MAX
                           * PS2000_GEN_DDS_REF_CLK;
        double freq_inc = (double)delta_phase_increment / PS2000_GEN_PHASE_ACC_MAX
                          * PS2000_GEN_DDS_REF_CLK;

        _ps2000_gen_sweep_init(start_freq, stop_freq, freq_inc,
                               (double)dwell_count / PS2000_GEN_DDS_REF_CLK,
                               sweep_type, sweeps);
    } else {
        g_sweep_state.active = false;
    }

    /* Upload waveform data via bulk transfer */
    ctx->siggen.awg_buffer_len = (uint16_t)waveform_size;
    status = _ps2000_usb_bulk_transfer(ctx,
                                       PS2000_USB_EP_OUT,
                                       waveform,
                                       (uint32_t)waveform_size);
    if (status == PS2000_OK) {
        PS2000_LOG_INF("AWG uploaded: %d bytes", waveform_size);
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
 * Device-independent helper used for preview and self-test.
 * FIX #200: Uses corrected wave type numbering.
 */
ps2000_status_t ps2000_gen_create_waveform(double *buffer,
                                           uint32_t num_samples,
                                           uint32_t wave_type) {
    if (!buffer || num_samples == 0U) return PS2000_ERR_INVALID_PARAM;
    if (wave_type > PS2000_GEN_WAVE_HALF_SINE) return PS2000_ERR_SIGGEN_CFG;

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

    /* Disable sweep */
    g_sweep_state.active = false;

    ctx->siggen.freq_hz = 0.0;
    ctx->siggen.phase = 0.0;
    ctx->siggen.awg_buffer_len = 0;
    ctx->siggen.config = PS2000_SIGGEN_DEFAULT;

    PS2000_LOG_INF("Signal generator disabled");
    pthread_mutex_unlock(&ctx->sync.mutex);
    return PS2000_OK;
}

/**
 * @brief Advance sweep state (called from streaming/block capture loop).
 *
 * This function should be called periodically to update the sweep
 * frequency. It returns the current phase increment for the DDS.
 *
 * @param ctx       Device context
 * @param dt        Time elapsed since last call (seconds)
 * @param phase_inc Output: updated phase increment
 * @return true if sweep is active, false if completed or inactive
 */
bool _ps2000_gen_sweep_update(ps2000_context_t *ctx, double dt,
                              uint32_t *phase_inc) {
    if (!ctx || !phase_inc) return false;
    if (!g_sweep_state.active) return false;

    double new_freq;
    _ps2000_gen_sweep_step(dt, &new_freq);
    *phase_inc = _ps2000_gen_calc_phase_inc(new_freq);
    ctx->siggen.freq_hz = new_freq;

    return g_sweep_state.active;
}