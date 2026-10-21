/****************************************************************************
* ps2000_info_ui.c — Реализация информационных и UI функций
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>
#include <stdio.h>

extern ps2000_global_state_t g_ps2000_state;

static ps2000_context_t* _ps2000_get_context(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return NULL;
    return g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
}

int16_t ps2000_get_unit_info(int16_t handle,
                             int8_t *string,
                             int16_t string_length,
                             int16_t line) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag || !string || string_length <= 0) return 0;

    const char *info_str = NULL;
    switch ((PS2000_INFO)line) {
        case PS2000_DRIVER_VERSION: info_str = PS2000_DRIVER_VERSION; break;
        case PS2000_USB_VERSION: info_str = "2.0"; break;
        case PS2000_HARDWARE_VERSION: info_str = "1.0"; break;
        case PS2000_VARIANT_INFO:
            info_str = ctx->info.variant_string;
            if (info_str[0] == '\0') info_str = "PS2000";
            break;
        case PS2000_BATCH_AND_SERIAL:
            info_str = ctx->info.serial_string;
            if (info_str[0] == '\0') info_str = "0/0";
            break;
        case PS2000_CAL_DATE: info_str = "Not Specified"; break;
        case PS2000_ERROR_CODE: info_str = "0"; break;
        case PS2000_KERNEL_DRIVER_VERSION: info_str = "libusb-1.0"; break;
        case PS2000_DRIVER_PATH: info_str = ""; break;
        default: return 0;
    }

    int16_t copy_len = (int16_t)strlen(info_str);
    if (copy_len >= string_length) {
        copy_len = string_length - 1;
    }
    memcpy(string, info_str, copy_len);
    string[copy_len] = '\0';
    return 1;
}

int16_t ps2000_get_timebase(int16_t handle,
                            int16_t timebase,
                            int32_t no_of_samples,
                            int32_t *time_interval,
                            int16_t *time_units,
                            int16_t oversample,
                            int32_t *max_samples) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    if (timebase < 0 || timebase > PS2000_MAX_TIMEBASE) return 0;

    double interval_ns = _ps2000_math_timebase_interval((uint32_t)timebase);
    if (oversample > 1) {
        interval_ns *= (double)oversample;
    }

    if (time_interval) {
        *time_interval = (int32_t)_ps2000_math_round_fast(interval_ns);
    }
    if (time_units) {
        *time_units = (int16_t)PS2000_NS;
    }
    if (max_samples) {
        uint32_t hw_limit = ctx->capture.buffer_size;
        if (hw_limit == 0 || hw_limit > PS2000_OSC_MAX_SAMPLES) {
            hw_limit = PS2000_OSC_MAX_SAMPLES;
        }
        *max_samples = (int32_t)hw_limit;
    }
    return 1;
}

int32_t ps2000_set_ets(int16_t handle,
                       int16_t mode,
                       int16_t ets_cycles,
                       int16_t ets_interleave) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return (int32_t)PS2000_NOT_FOUND;

    if (mode == PS2000_ETS_OFF) {
        ctx->capture.decimation_divisor = 1.0;
        return (int32_t)PS2000_OK;
    }
    if (ets_cycles < 1 || ets_interleave < 1) {
        return (int32_t)PS2000_INVALID_PARAMETER; // Note: This enum value might need definition if not in ps2000.h
    }

    int32_t ratio = (int32_t)ets_cycles / (int32_t)ets_interleave;
    if (ratio < PS2000_MIN_ETS_CYCLES_INTERLEAVE_RATIO ||
        ratio > PS2000_MAX_ETS_CYCLES_INTERLEAVE_RATIO) {
        return (int32_t)PS2000_INVALID_PARAMETER;
    }

    ctx->capture.decimation_divisor = (double)ets_interleave;

    ps2000_status_t status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                                               0x20, (uint16_t)mode);
    if (status != PS2000_OK) return (int32_t)PS2000_CONFIG_FAIL;

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x21, (uint16_t)ets_cycles);
    if (status != PS2000_OK) return (int32_t)PS2000_CONFIG_FAIL;

    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                               0x22, (uint16_t)ets_interleave);
    return (status == PS2000_OK) ? (int32_t)PS2000_OK : (int32_t)PS2000_CONFIG_FAIL;
}

int16_t ps2000_flash_led(int16_t handle) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    ps2000_status_t status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                                               0x30, 0x01);
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_set_led(int16_t handle, int16_t state) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    ps2000_status_t status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                                               0x31, (uint16_t)(state ? 1 : 0));
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_set_light(int16_t handle, int16_t state) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    ps2000_status_t status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
                                               0x32, (uint16_t)(state ? 1 : 0));
    return (status == PS2000_OK) ? 1 : 0;
}

int16_t ps2000_last_button_press(int16_t handle) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    uint16_t button_reg = 0;
    ps2000_status_t status = _ps2000_hw_read_reg(ctx, 0x33, &button_reg);
    if (status != PS2000_OK) return 0;
    return (int16_t)(button_reg & 0x03);
}

int16_t ps2000_open_unit_async(void) {
    ps2000_status_t status = _ps2000_init_global_state();
    if (status != PS2000_OK) return (int16_t)PS2000_OPEN_PROGRESS_FAIL;
    return (int16_t)PS2000_OPEN_PROGRESS_PENDING;
}

int16_t ps2000_open_unit_progress(int16_t *handle,
                                  int16_t *progress_percent) {
    if (!handle || !progress_percent) return (int16_t)PS2000_OPEN_PROGRESS_FAIL;
    for (uint32_t i = 0; i < PS2000_MAX_UNITS; i++) {
        if (g_ps2000_state.contexts[i] && g_ps2000_state.contexts[i]->active_flag) {
            *handle = (int16_t)(i + PS2000_FIRST_USB);
            *progress_percent = 100;
            return (int16_t)PS2000_OPEN_PROGRESS_COMPLETE;
        }
    }
    *progress_percent = 0;
    return (int16_t)PS2000_OPEN_PROGRESS_PENDING;
}

int16_t ps2000PingUnit(int16_t handle) {
    ps2000_context_t *ctx = _ps2000_get_context(handle);
    if (!ctx || !ctx->active_flag) return 0;
    uint16_t status_reg = 0;
    ps2000_status_t status = _ps2000_hw_read_reg(ctx, 0x00, &status_reg);
    return (status == PS2000_OK) ? 1 : 0;
}