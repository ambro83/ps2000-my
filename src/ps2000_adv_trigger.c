/****************************************************************************
* ps2000_adv_trigger.c — Реализация Advanced Trigger функций
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>

extern ps2000_global_state_t g_ps2000_state;

static ps2000_context_t* _ps2000_get_context(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return NULL;
    return g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
}

int16_t ps2000SetAdvTriggerChannelProperties(int16_t handle,
                                             PS2000_TRIGGER_CHANNEL_PROPERTIES *channelProperties,
                                             int16_t nChannelProperties,
                                             int32_t autoTriggerMilliseconds) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    if (!channelProperties || nChannelProperties <= 0) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }

    for (int16_t i = 0; i < nChannelProperties && i < PS2000_MAX_CHANNELS; i++) {
        uint16_t ch = (uint16_t)channelProperties[i].channel;
        if (ch >= PS2000_MAX_CHANNELS) continue;
        ctx->capture.trigger_threshold = (uint16_t)channelProperties[i].thresholdMajor;
        ctx->capture.trig_alt_param_a = (uint16_t)channelProperties[i].thresholdMinor;
        ctx->capture.trig_alt_param_b = (uint16_t)channelProperties[i].hysteresis;
    }

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x10, (uint16_t)(autoTriggerMilliseconds & 0xFFFF));
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000SetAdvTriggerChannelConditions(int16_t handle,
                                             PS2000_TRIGGER_CONDITIONS *conditions,
                                             int16_t nConditions) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    if (!conditions || nConditions <= 0) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }

    uint16_t trigger_mask = 0;
    if (conditions[0].channelA == PS2000_CONDITION_TRUE) trigger_mask |= 0x01;
    if (conditions[0].channelB == PS2000_CONDITION_TRUE) trigger_mask |= 0x02;
    if (conditions[0].channelC == PS2000_CONDITION_TRUE) trigger_mask |= 0x04;
    if (conditions[0].channelD == PS2000_CONDITION_TRUE) trigger_mask |= 0x08;
    if (conditions[0].external == PS2000_CONDITION_TRUE) trigger_mask |= 0x10;

    ctx->capture.channel_mask = (uint8_t)trigger_mask;

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x11, trigger_mask);
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000SetAdvTriggerChannelDirections(int16_t handle,
                                             PS2000_THRESHOLD_DIRECTION channelA,
                                             PS2000_THRESHOLD_DIRECTION channelB,
                                             PS2000_THRESHOLD_DIRECTION channelC,
                                             PS2000_THRESHOLD_DIRECTION channelD,
                                             PS2000_THRESHOLD_DIRECTION ext) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    uint16_t directions = ((uint16_t)channelA & 0x07) |
                          (((uint16_t)channelB & 0x07) << 3) |
                          (((uint16_t)channelC & 0x07) << 6) |
                          (((uint16_t)channelD & 0x07) << 9) |
                          (((uint16_t)ext & 0x07) << 12);

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x12, directions);
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000SetPulseWidthQualifier(int16_t handle,
                                     PS2000_PWQ_CONDITIONS *conditions,
                                     int16_t nConditions,
                                     PS2000_THRESHOLD_DIRECTION direction,
                                     uint32_t lower,
                                     uint32_t upper,
                                     PS2000_PULSE_WIDTH_TYPE type) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x13, (uint16_t)(lower & 0xFFFF));
    if (status != PS2000_OK) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x14, (uint16_t)(upper & 0xFFFF));
    if (status != PS2000_OK) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }

    uint16_t pwq_config = ((uint16_t)type & 0x07) |
                          (((uint16_t)direction & 0x07) << 3);
    
    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x15, pwq_config);
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000SetAdvTriggerDelay(int16_t handle,
                                 uint32_t delay,
                                 float preTriggerDelay) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    ctx->capture.pre_trigger = delay;

    if (ctx->capture.sample_interval > 0.0 && preTriggerDelay > 0.0f) {
        double delay_samples = (double)preTriggerDelay /
                               (ctx->capture.sample_interval / 1e9);
        int32_t delay_int = (int32_t)_ps2000_math_round_fast(delay_samples);
        if (delay_int > (int32_t)ctx->capture.buffer_size) {
            delay_int = (int32_t)ctx->capture.buffer_size;
        }
        ctx->capture.trig_alt_param_a = (uint16_t)delay_int;
    }

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x16, (uint16_t)(delay & 0xFFFF));
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}