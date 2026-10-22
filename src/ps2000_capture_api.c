/****************************************************************************
* ps2000_capture_api.c — Реализация функций управления сбором данных
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>

extern ps2000_global_state_t g_ps2000_state;

static ps2000_context_t* _ps2000_get_context(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return NULL;
    return g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
}

int16_t ps2000_set_trigger2(int16_t handle,
                            int16_t source,
                            int16_t threshold,
                            int16_t direction,
                            float delay,
                            int16_t auto_trigger_ms) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    if (source < PS2000_CHANNEL_A || source > PS2000_EXTERNAL) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }
    if (direction != PS2000_RISING && direction != PS2000_FALLING) {
        pthread_mutex_unlock(&ctx->sync.mutex);
        return 0;
    }

    ctx->capture.trigger_enabled = 1;
    ctx->capture.trigger_threshold = (uint16_t)threshold;
    ctx->capture.trig_alt_channel = (uint16_t)source;

    if (ctx->capture.sample_interval > 0.0) {
        double delay_samples = (double)delay / (ctx->capture.sample_interval / 1e9);
        int32_t delay_int = (int32_t)_ps2000_math_round_fast(delay_samples);
        if (delay_int < 0) delay_int = 0;
        if (delay_int > (int32_t)ctx->capture.buffer_size) {
            delay_int = (int32_t)ctx->capture.buffer_size;
        }
        ctx->capture.pre_trigger = (uint32_t)delay_int;
    }

    /* FIX #1: Check if ops delegate exists, otherwise use direct HW call */
    if (ctx->ops.set_trigger) {
        status = (ps2000_status_t)ctx->ops.set_trigger(ctx, 
            ((uint16_t)source & 0x07) |
            (((uint16_t)direction & 0x01) << 3) |
            (((uint16_t)threshold & 0x0FFF) << 4), 
            (uint16_t)auto_trigger_ms);
    } else {
        uint16_t trig_config = ((uint16_t)source & 0x07) |
                               (((uint16_t)direction & 0x01) << 3) |
                               (((uint16_t)threshold & 0x0FFF) << 4);
        status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_ADC_SPI,
                                   (uint16_t)trig_config, (uint16_t)auto_trigger_ms);
    }

    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_set_trigger(int16_t handle,
                           int16_t source,
                           int16_t threshold,
                           int16_t direction,
                           int16_t delay,
                           int16_t auto_trigger_ms) {
    return ps2000_set_trigger2(handle, source, threshold, direction,
                               (float)delay, auto_trigger_ms);
}

int16_t ps2000_ready(int16_t handle) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    if (ctx->capture.capture_state == PS2000_DEVICE_STATE_SPECIAL) {
        return 1;
    }

    uint16_t status_reg = 0;
    ps2000_status_t status = _ps2000_hw_read_reg(ctx, 0x00, &status_reg);
    if (status != PS2000_OK) return 0;

    if (status_reg & 0x0020) {
        ctx->capture.capture_state = PS2000_DEVICE_STATE_SPECIAL;
        return 1;
    }
    return 0;
}

int16_t ps2000_stop(int16_t handle) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    ctx->capture.capture_state = 0;
    ctx->streaming.stream_write_pos = 0;
    ctx->capture.trigger_enabled = 0;
    ctx->capture.trigger_threshold = 0;

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_PACKET, 0, 0);
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

/* FIX #3: Simplified run_block to delegate heavy lifting to ops table */
int16_t ps2000_run_block(int16_t handle,
                         int32_t no_of_values,
                         int16_t timebase,
                         int16_t oversample,
                         int32_t *time_indisposed_ms) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    /* FIX #3: Delegate to ops table if available, matching original asm behavior */
    if (ctx->ops.run_block) {
        /* The ops->run_block is expected to handle timebase calc and buffer setup internally */
        status = (ps2000_status_t)ctx->ops.run_block(ctx, 
                                                     (uint32_t)no_of_values, 
                                                     (uint32_t)timebase);
        
        /* Calculate indisposed time manually if not returned by ops */
        if (time_indisposed_ms) {
             double interval_ns = _ps2000_math_timebase_interval((uint32_t)timebase);
             if (oversample > 1) interval_ns *= (double)oversample;
             *time_indisposed_ms = (int32_t)((double)no_of_values * interval_ns / 1e6);
        }
    } else {
        status = PS2000_ERR_NOT_IMPLEMENTED;
    }

    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int32_t ps2000_get_values(int16_t handle,
                          int16_t *buffer_a,
                          int16_t *buffer_b,
                          int16_t *buffer_c,
                          int16_t *buffer_d,
                          int16_t *overflow,
                          int32_t no_of_values) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    int16_t *buffers[] = { buffer_a, buffer_b, buffer_c, buffer_d };
    int32_t total_read = 0;

    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        if (!buffers[ch]) continue;
        if ((ctx->capture.channel_mask & (1U << ch)) == 0U) continue;
        
        int16_t *source = ctx->capture.channel_buffers[ch];
        if (!source) continue;

        uint32_t count = (uint32_t)no_of_values;
        if (count > ctx->capture.buffer_size) {
            count = ctx->capture.buffer_size;
        }

        double gain = ctx->info.cal_gain[ch];
        double offset = (double)ctx->info.cal_offset[ch];
        
        for (uint32_t i = 0; i < count; i++) {
            buffers[ch][i] = _ps2000_math_adc_convert(source[i], gain, offset);
        }

        if (overflow) {
            overflow[ch] = 0;
            for (uint32_t i = 0; i < count; i++) {
                if (source[i] == PS2000_MIN_VALUE) {
                    overflow[ch] |= (1 << ch);
                    break;
                }
            }
        }
        if ((int32_t)count > total_read) {
            total_read = (int32_t)count;
        }
    }

    ctx->capture.capture_state = 0;
    pthread_mutex_unlock(&ctx->sync.mutex);
    return total_read;
}