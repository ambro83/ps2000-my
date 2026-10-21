/****************************************************************************
* ps2000_siggen_api.c — Wrapper функции для генератора сигналов
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>
#include <math.h>

extern ps2000_global_state_t g_ps2000_state;

static ps2000_context_t* _ps2000_get_context(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return NULL;
    return g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
}

int16_t ps2000_set_sig_gen_built_in(int16_t handle,
                                    int32_t offsetVoltage,
                                    uint32_t pkToPk,
                                    PS2000_WAVE_TYPE waveType,
                                    float startFrequency,
                                    float stopFrequency,
                                    float increment,
                                    float dwellTime,
                                    PS2000_SWEEP_TYPE sweepType,
                                    uint32_t sweeps) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    if ((uint32_t)waveType > PS2000_HALF_SINE) return 0;
    if (startFrequency < PS2000_MIN_SIGGEN_FREQ || startFrequency > PS2000_MAX_SIGGEN_FREQ) return 0;
    if (stopFrequency < PS2000_MIN_SIGGEN_FREQ || stopFrequency > PS2000_MAX_SIGGEN_FREQ) return 0;

    double offset_norm = (double)offsetVoltage / 1000000.0;
    double pkpk_norm = (double)pkToPk / 1000000.0;

    bool is_sweep = (fabsf(startFrequency - stopFrequency) > 0.001f) &&
                    (sweepType < MAX_SWEEP_TYPES);

    ps2000_status_t status;
    if (is_sweep && increment > 0.0f && dwellTime > 0.0f) {
        status = _ps2000_gen_set_builtin_internal(
            ctx, offset_norm, pkpk_norm, (uint32_t)waveType,
            (double)startFrequency, (double)stopFrequency,
            (double)increment, (double)dwellTime,
            (uint32_t)sweepType, sweeps
        );
    } else {
        status = _ps2000_gen_set_builtin_internal(
            ctx, (double)startFrequency, (uint32_t)waveType,
            pkpk_norm / 2.0, offset_norm
        );
    }
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_set_sig_gen_arbitrary(int16_t handle,
                                     int32_t offsetVoltage,
                                     uint32_t pkToPk,
                                     uint32_t startDeltaPhase,
                                     uint32_t stopDeltaPhase,
                                     uint32_t deltaPhaseIncrement,
                                     uint32_t dwellCount,
                                     uint8_t *arbitraryWaveform,
                                     int32_t arbitraryWaveformSize,
                                     PS2000_SWEEP_TYPE sweepType,
                                     uint32_t sweeps) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    if (!arbitraryWaveform || arbitraryWaveformSize <= 0) return 0;
    if ((uint32_t)arbitraryWaveformSize > PS2000_AWG_MAX_SAMPLES) return 0;

    ps2000_status_t status = _ps2000_gen_set_arbitrary_internal(
        ctx, offsetVoltage, pkToPk,
        startDeltaPhase, stopDeltaPhase,
        deltaPhaseIncrement, dwellCount,
        arbitraryWaveform, arbitraryWaveformSize,
        (uint32_t)sweepType, sweeps
    );
    return (status == PS2000_OK) ? 1 : 0;
}