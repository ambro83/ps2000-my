#include "ps2000_internal.h"
/* libusb API available via ps2000_internal.h (FIX #10: explicit note) */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ========================================================================== */
/*                     FIRMWARE LOADER (Anchor Protocol)                      */
/* ========================================================================== */

#define ANCHOR_CMD_RAM_WRITE      0xA0U
#define ANCHOR_CPUCS_REG          0xE600U
#define ANCHOR_CPUCS_HALT         0x01U
#define ANCHOR_CPUCS_RUN          0x00U
#define PS2000_FW_ANCHOR_CHUNK    4096U

/**
 * @brief Write data to Anchor chip RAM via USB Control Transfer.
 *
 * Reconstructs "AnchorWrite download %llx returned err" (0x4E68F).
 */
static ps2000_status_t _ps2000_fw_anchor_write_chunk(ps2000_context_t *ctx,
                                                     uint16_t addr,
                                                     const uint8_t *data,
                                                     uint16_t len) {
    if (!ctx || !ctx->usb_host.handle || !data) return PS2000_ERR_INVALID_PARAM;

    int transferred = libusb_control_transfer(
        ctx->usb_host.handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        ANCHOR_CMD_RAM_WRITE,
        addr, 0,
        (uint8_t*)data, len,
        PS2000_USB_TIMEOUT_MS
    );

    if (transferred < 0) {
        PS2000_LOG_ERR("AnchorWrite download %04X returned err %d", addr, transferred);
        return PS2000_ERR_USB;
    }

    return PS2000_OK;
}

static ps2000_status_t _ps2000_fw_anchor_set_cpu(ps2000_context_t *ctx, bool halt) {
    uint8_t cpucs_val = halt ? ANCHOR_CPUCS_HALT : ANCHOR_CPUCS_RUN;
    return _ps2000_fw_anchor_write_chunk(ctx, ANCHOR_CPUCS_REG, &cpucs_val, 1);
}

ps2000_status_t _ps2000_fw_load(ps2000_context_t *ctx,
                                const uint8_t *fw_data, uint32_t fw_len) {
    if (!ctx || !fw_data || fw_len == 0) return PS2000_ERR_INVALID_PARAM;

    PS2000_LOG_INF("Loading firmware via Anchor protocol (%u bytes)...", fw_len);

    ps2000_status_t status = _ps2000_fw_anchor_set_cpu(ctx, true);
    if (status != PS2000_OK) return status;

    uint32_t offset = 0;
    while (offset < fw_len) {
        uint32_t chunk_len = fw_len - offset;
        if (chunk_len > PS2000_FW_ANCHOR_CHUNK) chunk_len = PS2000_FW_ANCHOR_CHUNK;

        status = _ps2000_fw_anchor_write_chunk(ctx, (uint16_t)offset,
                                               &fw_data[offset], (uint16_t)chunk_len);
        if (status != PS2000_OK) {
            _ps2000_fw_anchor_set_cpu(ctx, false);
            return status;
        }

        offset += chunk_len;
    }

    status = _ps2000_fw_anchor_set_cpu(ctx, false);
    if (status == PS2000_OK) {
        usleep(100000); /* Wait for re-enumeration */
        PS2000_LOG_INF("Firmware loaded and CPU started");
    }

    return status;
}

ps2000_status_t _ps2000_fw_init(ps2000_context_t *ctx) {
    (void)ctx;
    return PS2000_OK;
}

void _ps2000_fw_cleanup(ps2000_context_t *ctx) {
    (void)ctx;
}

ps2000_status_t _ps2000_fw_reset_device(ps2000_context_t *ctx) {
    if (!ctx || !ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    uint8_t cmd = 0x01;
    return _ps2000_usb_control_transfer(ctx,
                                        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
                                        0xA8, 0, 0, &cmd, 1);
}

ps2000_status_t _ps2000_fw_verify(ps2000_context_t *ctx) {
    (void)ctx;
    return PS2000_OK;
}