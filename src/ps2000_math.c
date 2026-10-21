#include "ps2000_internal.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* ========================================================================== */
/*                     FAST ROUNDING (2^52 magic number)                      */
/* ========================================================================== */

/**
 * @brief Fast round-to-nearest using the 2^52 bias trick.
 *
 * Reconstructs the SSE2 pattern found in sub_203A0, sub_3AA64, sub_38CB8:
 *   movapd  xmm1, xmm0
 *   andpd   xmm1, DOUBLE_ABS_MASK      ; extract |x|
 *   ucomisd xmm1, FAST_ROUND_MAGIC     ; compare |x| with 2^52
 *   jae     .already_integer           ; |x| >= 2^52 means already integer
 *   addsd   xmm0, FAST_ROUND_MAGIC     ; add bias (forces FPU rounding)
 *   subsd   xmm0, FAST_ROUND_MAGIC     ; remove bias
 *
 * This is the classic IEEE754 fast-round idiom: adding 2^52 forces the FPU
 * to drop the fractional part due to mantissa width limits (52 bits).
 * The result is rounded to nearest (ties to even).
 */
double _ps2000_math_round_fast(double val) {
    const double magic = PS2000_FAST_ROUND_MAGIC; /* 2^52 */
    /* Extract absolute value using bit manipulation (andpd pattern) */
    uint64_t bits;
    memcpy(&bits, &val, sizeof(bits));
    bits &= PS2000_DOUBLE_ABS_MASK;
    double abs_val;
    memcpy(&abs_val, &bits, sizeof(abs_val));

    /* Check if |val| >= 2^52 (already integer or inf/nan) */
    if (abs_val >= magic) {
        return val;
    }
    /* The actual rounding: (val + 2^52) - 2^52 */
    double rounded = (val + magic) - magic;
    return rounded;
}

/* ========================================================================== */
/*                     ADC NORMALIZATION (14-bit ADC)                         */
/* ========================================================================== */

/**
 * @brief Normalize raw 14-bit ADC value to [-1.0, +1.0] range.
 *
 * Reconstructs sub_F9E4 / sub_F9A0:
 *   cvtsi2sd xmm0, raw
 *   divsd    xmm0, ADC_FULL_SCALE      ; divide by 16384.0
 *
 * The PS2000 uses a 14-bit ADC; raw values are normalized by dividing
 * by ADC_FULL_SCALE (16384.0).
 */
double _ps2000_math_adc_normalize(int16_t raw) {
    return (double)raw / PS2000_ADC_FULL_SCALE;
}

/**
 * @brief Convert normalized value back to raw ADC counts.
 *
 * Uses ADC_FULL_SCALE (16384.0) as the multiplier.
 */
int16_t _ps2000_math_adc_denormalize(double normalized) {
    double val = normalized * PS2000_ADC_FULL_SCALE;
    double rounded = _ps2000_math_round_fast(val);
    int32_t result = (int32_t)(int64_t)rounded;
    /* Clamp to int16 range */
    if (result > 32767) result = 32767;
    if (result < -32768) result = -32768;
    return (int16_t)result;
}

/* ========================================================================== */
/*                     TIMEBASE CALCULATION                                   */
/* ========================================================================== */

/**
 * @brief Calculate timebase interval in nanoseconds.
 *
 * Reconstructs sub_10BF0 / sub_2AC11 using constants:
 *   TIMEBASE_LOG_BASE (10.0) - logarithmic scale base
 *   TIMEBASE_NS_PER_SEC (1e9) - seconds to nanoseconds
 *   TIMEBASE_MAX_INTERVAL (INT32_MAX) - upper clamp
 *   TIMEBASE_ROUND_BIAS (0.5) - rounding correction
 *
 * The PS2000 uses a 1-2-5 timebase sequence typical for oscilloscopes.
 */
double _ps2000_math_timebase_interval(uint32_t index) {
    /* Base interval calculation: 10^(index * 0.5) scaled to nanoseconds */
    double exponent = (double)index * PS2000_TIMEBASE_ROUND_BIAS;
    double interval_s = pow(PS2000_TIMEBASE_LOG_BASE, exponent);
    /* Convert to nanoseconds */
    double interval_ns = interval_s * PS2000_TIMEBASE_NS_PER_SEC;
    /* Apply unit step correction */
    interval_ns *= PS2000_TIMEBASE_UNIT_STEP;
    /* Clamp to INT32_MAX (from qword_A4C50 comparison in original) */
    if (interval_ns > PS2000_TIMEBASE_MAX_INTERVAL) {
        interval_ns = PS2000_TIMEBASE_MAX_INTERVAL;
    }
    /* Round to nearest integer nanosecond */
    interval_ns = _ps2000_math_round_fast(interval_ns);
    return interval_ns;
}

/**
 * @brief Timing/trigger position calculation.
 *
 * Reconstructs sub_10BF0 with exact constants:
 *   cvtsi2sd -> mulsd -> divsd -> addsd(ROUND_BIAS) -> mulsd -> cvttsd2si
 */
int32_t _ps2000_math_timing_calc(int32_t raw_val, double interval, double offset) {
    double val = (double)raw_val * interval;
    /* Division by percent scale (100.0) for percentage-based delays */
    val /= PS2000_TIMEBASE_PERCENT_SCALE;
    /* Apply user offset */
    val += offset;
    /* Round-to-nearest with 0.5 bias (TIMEBASE_ROUND_BIAS) */
    val += PS2000_TIMEBASE_ROUND_BIAS;
    /* Truncate to integer (cvttsd2si semantics) */
    int32_t result = (int32_t)(int64_t)val;
    /* Clamp to valid range */
    if (result > (int32_t)PS2000_TIMEBASE_MAX_INTERVAL) {
        result = (int32_t)PS2000_TIMEBASE_MAX_INTERVAL;
    }
    return result;
}

/* ========================================================================== */
/*                     ADC CONVERSION WITH CALIBRATION                        */
/* ========================================================================== */

/**
 * @brief Full ADC conversion chain with calibration.
 *
 * FIX #7: Reconstructed from sub_F9E4 with correct formula:
 *
 *   1. Normalize: normalized = raw / 16384.0
 *   2. Apply gain: gained = normalized * gain
 *   3. Add offset: calibrated = gained + corr_offset
 *   4. Scale to int16: scaled = calibrated * 32768.0
 *   5. Round and clamp to [-32768, 32767]
 *
 * Original assembly (sub_F9E4):
 *   cvtsi2sd xmm0, raw
 *   divsd    xmm0, cs:qword_16384     ; normalize to [-1, +1]
 *   mulsd    xmm0, gain               ; apply calibrated gain
 *   addsd    xmm0, corr_offset        ; add correction offset
 *   mulsd    xmm0, cs:qword_32768     ; scale to int16 range
 *   cvttsd2si eax, xmm0               ; truncate
 *   lea      edx, [rax+7FFEh]         ; range check trick
 *   cmp      edx, 0FFFCh
 *   jbe      no_clamp
 */
int16_t _ps2000_math_adc_convert(int16_t raw, double gain, double corr_offset) {
    /* Step 1: Normalize raw value to [-1.0, +1.0] */
    double normalized = (double)raw / PS2000_ADC_FULL_SCALE;
    /* Step 2: Apply calibrated gain */
    double gained = normalized * gain;
    /* Step 3: Add correction offset */
    double calibrated = gained + corr_offset;
    /* Step 4: Scale back to int16 range */
    double scaled = calibrated * PS2000_ADC_INT16_SCALE;
    
    /* Step 5: Round to nearest integer */
    double rounded = _ps2000_math_round_fast(scaled);
    
    /* FIX #7: Clamp in double domain BEFORE integer cast to avoid UB */
    if (rounded > 32767.0) rounded = 32767.0;
    if (rounded < -32768.0) rounded = -32768.0;
    
    int32_t converted = (int32_t)(int64_t)rounded;
    
    return (int16_t)converted;
}

/* ========================================================================== */
/*                     STREAMING POSITION CALCULATION                         */
/* ========================================================================== */

/**
 * @brief Streaming buffer position with 256-byte alignment.
 *
 * Reconstructs sub_10770, sub_104D4:
 *   cvtsi2sd -> mulsd -> cvttsd2si -> add 0FFh -> and ~0FFh
 */
uint32_t _ps2000_math_stream_pos(uint32_t base, uint32_t increment, double scale) {
    double scaled_inc = (double)increment * scale;
    uint32_t int_inc = (uint32_t)(int64_t)_ps2000_math_round_fast(scaled_inc);
    uint32_t aligned = base + int_inc;
    /* Round UP to next 256-byte boundary */
    aligned = (aligned + 0xFFU) & 0xFFFFFF00U;
    return aligned;
}

/* ========================================================================== */
/*                     AWG / DAC CONVERSION                                   */
/* ========================================================================== */

/**
 * @brief Convert normalized AWG value [-1.0, +1.0] to 12-bit DAC code.
 *
 * Reconstructs sub_28450 using DAC_FULL_SCALE (4095.5) and
 * DAC_MID_SCALE (2047.0):
 *   dac_code = (normalized * DAC_MID_SCALE) + DAC_MID_SCALE + 1
 *   Clamped to [0, 4095]
 */
double _ps2000_math_awg_to_dac(double normalized) {
    /* Scale to DAC range: [-1, 1] -> [0, 4095] */
    double dac_val = (normalized * PS2000_DAC_MID_SCALE) + PS2000_DAC_MID_SCALE + 1.0;
    /* Round to nearest DAC code */
    dac_val = _ps2000_math_round_fast(dac_val);
    /* Clamp to valid 12-bit range */
    if (dac_val > PS2000_DAC_FULL_SCALE) dac_val = PS2000_DAC_FULL_SCALE;
    if (dac_val < 0.0) dac_val = 0.0;
    return dac_val;
}

/**
 * @brief Convert 12-bit DAC code back to normalized value.
 *
 * Inverse of _ps2000_math_awg_to_dac:
 *   normalized = (dac_code - DAC_MID_SCALE) / DAC_MID_SCALE
 */
double _ps2000_math_dac_to_awg(uint16_t dac_code) {
    double val = (double)dac_code - PS2000_DAC_MID_SCALE;
    return val / PS2000_DAC_MID_SCALE;
}

/* ========================================================================== */
/*                     SINE GENERATION (for AWG)                              */
/* ========================================================================== */

/**
 * @brief Generate sine wave sample using PI constant.
 *
 * The PS2000_MATH_PI constant is used in sub_285B0 for sine generation
 * in the AWG path.
 */
double _ps2000_math_sine_sample(uint32_t index, uint32_t total_samples) {
    if (total_samples == 0) return 0.0;
    double phase = (2.0 * PS2000_MATH_PI * (double)index) / (double)total_samples;
    return sin(phase);
}