/****************************************************************************
* ps2000_channel.c — Реализация ps2000_set_channel
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>

/*
* Range-to-gain mapping table.
*/
static const double ps2000_range_gain[] = {
    [PS2000_10MV]  = 50.0,
    [PS2000_20MV]  = 25.0,
    [PS2000_50MV]  = 10.0,
    [PS2000_100MV] = 5.0,
    [PS2000_200MV] = 2.5,
    [PS2000_500MV] = 1.0,
    [PS2000_1V]    = 0.5,
    [PS2000_2V]    = 0.25,
    [PS2000_5V]    = 0.1,
    [PS2000_10V]   = 0.05,
    [PS2000_20V]   = 0.025,
    [PS2000_50V]   = 0.01,
};

/*
* Range-to-SPI-value mapping for ADC configuration.
*/
static const uint16_t ps2000_range_spi[] = {
    [PS2000_10MV]  = 0x0001,
    [PS2000_20MV]  = 0x0002,
    [PS2000_50MV]  = 0x0004,
    [PS2000_100MV] = 0x0008,
    [PS2000_200MV] = 0x0010,
    [PS2000_500MV] = 0x0020,
    [PS2000_1V]    = 0x0040,
    [PS2000_2V]    = 0x0080,
    [PS2000_5V]    = 0x0100,
    [PS2000_10V]   = 0x0200,
    [PS2000_20V]   = 0x0400,
    [PS2000_50V]   = 0x0800,
};

static ps2000_status_t _ps2000_channel_configure(ps2000_context_t *ctx,
                                                 uint16_t channel,
                                                 int16_t enabled,
                                                 int16_t dc,
                                                 int16_t range) {
    if (range < 0 || range >= PS2000_MAX_RANGES) {
        return PS2000_ERR_INVALID_PARAM;
    }

    /* Update channel mask */
    if (enabled) {
        ctx->capture.channel_mask |= (1U << channel);
    } else {
        ctx->capture.channel_mask &= ~(1U << channel);
    }

    /* Configure ADC SPI for this channel */
    uint16_t spi_val = ps2000_range_spi[range];
    /* Add DC/AC coupling bit (bit 12) */
    if (dc) {
        spi_val |= 0x1000;
    }

    ps2000_status_t status = _ps2000_hw_adc_spi_write(ctx, (uint8_t)channel, spi_val);
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("ADC SPI write failed for ch %u: %d", channel, status);
        return status;
    }

    /* Update calibration gain for this range */
    ctx->info.cal_gain[channel] = ps2000_range_gain[range];
    
    PS2000_LOG_INF("Channel %u configured: enabled=%d dc=%d range=%d",
                   channel, enabled, dc, range);
    return PS2000_OK;
}

int16_t ps2000_set_channel(int16_t handle,
                           int16_t channel,
                           int16_t enabled,
                           int16_t dc,
                           int16_t range) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return 0;
    if (channel < PS2000_CHANNEL_A || channel > PS2000_EXTERNAL) return 0;
    if (range < PS2000_10MV || range >= PS2000_MAX_RANGES) return 0;

    extern ps2000_global_state_t g_ps2000_state;
    ps2000_context_t *ctx = g_ps2000_state.contexts[handle - PS2000_FIRST_USB];
    if (!ctx || !ctx->active_flag) return 0;

    ps2000_status_t status = _ps2000_util_mutex_lock_safe(
        &ctx->sync.mutex, PS2000_USB_TIMEOUT_MS);
    if (status != PS2000_OK) return 0;

    status = _ps2000_channel_configure(ctx, (uint16_t)channel, enabled, dc, range);
    
    pthread_mutex_unlock(&ctx->sync.mutex);
    return (status == PS2000_OK) ? 1 : 0;
}