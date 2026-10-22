/****************************************************************************
* ps2000_stream_api.c — Реализация Streaming API функций
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>

extern ps2000_global_state_t g_ps2000_state;

static ps2000_context_t* _ps2000_get_context(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return NULL;
    return g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
}

int16_t ps2000_run_streaming(int16_t handle,
                             int16_t sample_interval_ms,
                             int32_t max_samples,
                             int16_t windowed) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    if (sample_interval_ms <= 0 || max_samples <= 0) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    double interval_ns = (double)sample_interval_ms * 1e6;
    ctx->capture.sample_interval = interval_ns;

    if (!ctx->streaming.stream_buffer) {
        ctx->streaming.stream_buffer = calloc((size_t)max_samples * PS2000_MAX_CHANNELS,
                                              sizeof(int16_t));
        if (!ctx->streaming.stream_buffer) {
            pthread_mutex_unlock(&ctx->sync.mutex);
            return 0;
        }
        ctx->streaming.stream_buf_size = (uint32_t)max_samples;
    }
    ctx->streaming.stream_write_pos = 0;

    /* FIX #2: Delegate to ops table if available */
    uint16_t interval_us = (uint16_t)(sample_interval_ms * 1000);
    if (ctx->ops.start_streaming) {
        status = (ps2000_status_t)ctx->ops.start_streaming(ctx, interval_us, (uint32_t)max_samples, 0);
    } else {
        status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_DELAY_MS,
                                   interval_us, (uint16_t)(max_samples & 0xFFFF));
    }

    if (status == PS2000_OK) {
        PS2000_LOG_INF("Streaming started: interval=%dms max=%d",
                       sample_interval_ms, max_samples);
    }
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_run_streaming_ns(int16_t handle,
                                uint32_t sample_interval,
                                PS2000_TIME_UNITS time_units,
                                uint32_t max_samples,
                                int16_t auto_stop,
                                uint32_t noOfSamplesPerAggregate,
                                uint32_t overview_buffer_size) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    if (sample_interval == 0 || max_samples == 0) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    double interval_ns;
    switch (time_units) {
        case PS2000_FS: interval_ns = (double)sample_interval * 1e-6; break;
        case PS2000_PS: interval_ns = (double)sample_interval * 1e-3; break;
        case PS2000_NS: interval_ns = (double)sample_interval; break;
        case PS2000_US: interval_ns = (double)sample_interval * 1e3; break;
        case PS2000_MS: interval_ns = (double)sample_interval * 1e6; break;
        case PS2000_S:  interval_ns = (double)sample_interval * 1e9; break;
        default:
            pthread_mutex_unlock(&ctx->sync.mutex);
            return 0;
    }

    if (noOfSamplesPerAggregate > 1) {
        interval_ns *= (double)noOfSamplesPerAggregate;
    }
    ctx->capture.sample_interval = interval_ns;

    uint32_t buf_size = overview_buffer_size > 0 ? overview_buffer_size : max_samples;
    if (!ctx->streaming.stream_buffer || ctx->streaming.stream_buf_size < buf_size) {
        if (ctx->streaming.stream_buffer) free(ctx->streaming.stream_buffer);
        ctx->streaming.stream_buffer = calloc((size_t)buf_size * PS2000_MAX_CHANNELS,
                                              sizeof(int16_t));
        if (!ctx->streaming.stream_buffer) {
            pthread_mutex_unlock(&ctx->sync.mutex);
            return 0;
        }
        ctx->streaming.stream_buf_size = buf_size;
    }
    ctx->streaming.stream_write_pos = 0;

    uint32_t interval_int = (uint32_t)_ps2000_math_round_fast(interval_ns);
    
    /* FIX #2: Delegate to ops table if available */
    if (ctx->ops.start_streaming) {
         status = (ps2000_status_t)ctx->ops.start_streaming(ctx, 
                                                            (uint16_t)(interval_int & 0xFFFF), 
                                                            max_samples, 
                                                            auto_stop);
    } else {
        status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_DELAY_MS,
                                   (uint16_t)(interval_int & 0xFFFF),
                                   (uint16_t)(max_samples & 0xFFFF));
    }

    if (status == PS2000_OK) {
        PS2000_LOG_INF("Streaming (ns) started: interval=%uns max=%u agg=%u",
                       (uint32_t)interval_ns, max_samples, noOfSamplesPerAggregate);
    }
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_run_streaming_us(int16_t handle,
                                int32_t sample_interval_us,
                                int32_t max_samples,
                                int16_t windowed) {
    int16_t interval_ms = (int16_t)(sample_interval_us / 1000);
    if (interval_ms < 1) interval_ms = 1;
    return ps2000_run_streaming(handle, interval_ms, max_samples, windowed);
}

int16_t ps2000_get_streaming_last_values(int16_t handle,
                                         GetOverviewBuffersMaxMin callback) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag || !callback) return 0;
    if (!ctx->streaming.stream_buffer) return 0;

    int16_t *overview_buffers[PS2000_MAX_CHANNELS];
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        overview_buffers[ch] = ctx->capture.channel_buffers[ch];
    }
    uint32_t n_values = ctx->streaming.stream_write_pos;
    callback(overview_buffers, 0, 0, 0, 0, n_values);
    return 1;
}

int16_t ps2000_overview_buffer_status(int16_t handle,
                                      int16_t *previous_buffer_overrun) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    if (previous_buffer_overrun) {
        *previous_buffer_overrun = 0;
    }
    return 1;
}

uint32_t ps2000_get_streaming_values(int16_t handle,
                                     double *start_time,
                                     int16_t *pbuffer_a_max, int16_t *pbuffer_a_min,
                                     int16_t *pbuffer_b_max, int16_t *pbuffer_b_min,
                                     int16_t *pbuffer_c_max, int16_t *pbuffer_c_min,
                                     int16_t *pbuffer_d_max, int16_t *pbuffer_d_min,
                                     int16_t *overflow,
                                     uint32_t *triggerAt,
                                     int16_t *triggered,
                                     uint32_t no_of_values,
                                     uint32_t noOfSamplesPerAggregate) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    if (start_time) *start_time = 0.0;

    uint32_t available = ctx->streaming.stream_write_pos;
    uint32_t count = (no_of_values < available) ? no_of_values : available;

    int16_t *max_bufs[] = { pbuffer_a_max, pbuffer_b_max, pbuffer_c_max, pbuffer_d_max };
    int16_t *min_bufs[] = { pbuffer_a_min, pbuffer_b_min, pbuffer_c_min, pbuffer_d_min };

    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        if (!max_bufs[ch] || !min_bufs[ch]) continue;
        int16_t *source = ctx->capture.channel_buffers[ch];
        if (!source) continue;
        for (uint32_t i = 0; i < count; i++) {
            int16_t val = source[i];
            max_bufs[ch][i] = val;
            min_bufs[ch][i] = val;
        }
    }

    if (triggerAt) *triggerAt = 0;
    if (triggered) *triggered = 0;
    if (overflow) *overflow = 0;

    ctx->streaming.stream_write_pos -= count;
    pthread_mutex_unlock(&ctx->sync.mutex);
    return count;
}

uint32_t ps2000_get_streaming_values_no_aggregation(int16_t handle,
                                                    double *start_time,
                                                    int16_t *pbuffer_a,
                                                    int16_t *pbuffer_b,
                                                    int16_t *pbuffer_c,
                                                    int16_t *pbuffer_d,
                                                    int16_t *overflow,
                                                    uint32_t *triggerAt,
                                                    int16_t *trigger,
                                                    uint32_t no_of_values) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    if (start_time) *start_time = 0.0;

    uint32_t available = ctx->streaming.stream_write_pos;
    uint32_t count = (no_of_values < available) ? no_of_values : available;

    int16_t *buffers[] = { pbuffer_a, pbuffer_b, pbuffer_c, pbuffer_d };
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        if (!buffers[ch]) continue;
        int16_t *source = ctx->capture.channel_buffers[ch];
        if (!source) continue;
        memcpy(buffers[ch], source, count * sizeof(int16_t));
    }

    if (triggerAt) *triggerAt = 0;
    if (trigger) *trigger = 0;
    if (overflow) *overflow = 0;

    ctx->streaming.stream_write_pos -= count;
    pthread_mutex_unlock(&ctx->sync.mutex);
    return count;
}