#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/*                       INTERNAL USB CONSTANTS                               */
/* ========================================================================== */
/*
 * USB Vendor ID for Pico Technology.
 * Reconstructed from sub_4B797: mov ecx, 0CE9h
 */
#define PS2000_USB_VID            0x0CE9U

/*
 * USB Product ID for PS2000 series.
 * Note: Multiple PIDs may exist for different models.
 */
#define PS2000_USB_PID            0x1009U

/* USB interface and endpoint configuration */
#define PS2000_USB_INTERFACE      0
#define PS2000_USB_ALT_SETTING    1

/* FIX #13: PS2000_USB_EP_IN and PS2000_USB_EP_OUT are now defined
 * in ps2000_internal.h. Removed duplicate definitions here. */

/* ========================================================================== */
/*                          USB HELPERS                                       */
/* ========================================================================== */

/**
 * @brief Map libusb error codes to internal status codes.
 *
 * Provides a consistent error mapping layer between libusb
 * and the PS2000 internal error system.
 */
static ps2000_status_t _ps2000_usb_map_error(int libusb_err) {
    switch (libusb_err) {
        case LIBUSB_SUCCESS:            return PS2000_OK;
        case LIBUSB_ERROR_TIMEOUT:      return PS2000_ERR_TIMEOUT;
        case LIBUSB_ERROR_NO_DEVICE:    return PS2000_ERR_NOT_FOUND;
        case LIBUSB_ERROR_NOT_FOUND:    return PS2000_ERR_NOT_FOUND;
        case LIBUSB_ERROR_BUSY:         return PS2000_ERR_BUSY;
        case LIBUSB_ERROR_NO_MEM:       return PS2000_ERR_MEMORY;
        case LIBUSB_ERROR_INVALID_PARAM: return PS2000_ERR_INVALID_PARAM;
        case LIBUSB_ERROR_ACCESS:       return PS2000_ERR_NOT_OPENED;
        default:                        return PS2000_ERR_USB;
    }
}

/**
 * @brief Find a PS2000 device by serial number.
 *
 * Reconstructed from sub_4E3D9 / sub_4E420 device enumeration logic.
 * Iterates through all USB devices, checking VID/PID and optionally
 * matching the serial number.
 *
 * @param ctx    libusb context for enumeration
 * @param serial Serial number to match (0 = any device)
 * @return Opened device handle, or NULL if not found
 */
static libusb_device_handle* _ps2000_usb_find_device(libusb_context *ctx,
                                                     uint32_t serial) {
    libusb_device **dev_list = NULL;
    ssize_t cnt = libusb_get_device_list(ctx, &dev_list);
    if (cnt < 0) {
        PS2000_LOG_ERR("libusb_get_device_list failed: %zd", cnt);
        return NULL;
    }

    libusb_device_handle *found_handle = NULL;
    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) != 0) {
            continue;
        }
        /* Check VID/PID match */
        if (desc.idVendor != PS2000_USB_VID ||
            desc.idProduct != PS2000_USB_PID) {
            continue;
        }

        libusb_device_handle *handle = NULL;
        int ret = libusb_open(dev_list[i], &handle);
        if (ret != 0) {
            PS2000_LOG_DBG("Failed to open candidate device: %d", ret);
            continue;
        }

        /* Read serial number for verification */
        unsigned char sn_buf[32] = {0};
        ret = libusb_get_string_descriptor_ascii(
            handle, desc.iSerialNumber, sn_buf, sizeof(sn_buf));
        bool match = false;
        if (serial == 0) {
            /* Open first found device */
            match = true;
        } else if (ret > 0) {
            uint32_t dev_sn = 0;
            if (sscanf((char*)sn_buf, "%u", &dev_sn) == 1) {
                match = (dev_sn == serial);
            }
        }

        if (match) {
            found_handle = handle;
            break;
        }
        libusb_close(handle);
    }
    libusb_free_device_list(dev_list, 1);
    return found_handle;
}

/* ========================================================================== */
/*                       PUBLIC TRANSPORT API                                 */
/* ========================================================================== */

/**
 * @brief Open a PS2000 device by serial number.
 *
 * Performs the full USB open sequence:
 * 1. Initialize libusb context (if not already done)
 * 2. Find device by VID/PID and optional serial
 * 3. Claim interface
 * 4. Set alternate setting
 *
 * @param ctx    Device context to populate
 * @param serial Serial number (0 = first found device)
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_usb_open_device(ps2000_context_t *ctx,
                                        uint32_t serial) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (ctx->usb_host.handle) return PS2000_ERR_BUSY;

    /* Initialize libusb context on first open */
    if (!ctx->usb_host.ctx) {
        int ret = libusb_init(&ctx->usb_host.ctx);
        if (ret != LIBUSB_SUCCESS) {
            PS2000_LOG_ERR("libusb_init failed: %d", ret);
            return _ps2000_usb_map_error(ret);
        }
    }

    /* Find and open device */
    ctx->usb_host.handle = _ps2000_usb_find_device(ctx->usb_host.ctx, serial);
    if (!ctx->usb_host.handle) {
        PS2000_LOG_ERR("Device not found (serial=%u)", serial);
        return PS2000_ERR_NOT_FOUND;
    }

    /* Claim interface (from sub_4E3D9 pattern) */
    int ret = libusb_claim_interface(ctx->usb_host.handle,
                                     PS2000_USB_INTERFACE);
    if (ret != LIBUSB_SUCCESS) {
        PS2000_LOG_ERR("Failed to claim interface %d: %d",
                       PS2000_USB_INTERFACE, ret);
        libusb_close(ctx->usb_host.handle);
        ctx->usb_host.handle = NULL;
        return _ps2000_usb_map_error(ret);
    }

    /* Set alternate setting */
    ret = libusb_set_interface_alt_setting(
        ctx->usb_host.handle, PS2000_USB_INTERFACE, PS2000_USB_ALT_SETTING);
    if (ret != LIBUSB_SUCCESS) {
        PS2000_LOG_ERR("Failed to set alt setting %d: %d",
                       PS2000_USB_ALT_SETTING, ret);
        libusb_release_interface(ctx->usb_host.handle, PS2000_USB_INTERFACE);
        libusb_close(ctx->usb_host.handle);
        ctx->usb_host.handle = NULL;
        return _ps2000_usb_map_error(ret);
    }

    ctx->active_flag = 1;
    PS2000_LOG_INF("Device opened successfully (serial=%u)", serial);
    return PS2000_OK;
}

/**
 * @brief Close a PS2000 device and release all USB resources.
 *
 * Performs cleanup in reverse order of open:
 * 1. Release interface
 * 2. Close handle
 * 3. Free config descriptor
 * 4. Unref device
 * 5. Exit libusb context
 */
void _ps2000_usb_close_device(ps2000_context_t *ctx) {
    if (!ctx || !ctx->usb_host.handle) return;

    PS2000_LOG_INF("Closing device...");

    /* Release interface and close handle */
    libusb_release_interface(ctx->usb_host.handle, PS2000_USB_INTERFACE);
    libusb_close(ctx->usb_host.handle);
    ctx->usb_host.handle = NULL;
    ctx->active_flag = 0;

    /* Free config descriptor if allocated */
    if (ctx->usb_host.active_config) {
        libusb_free_config_descriptor(ctx->usb_host.active_config);
        ctx->usb_host.active_config = NULL;
    }

    /* Unref device if we have a reference */
    if (ctx->usb_host.dev) {
        libusb_unref_device(ctx->usb_host.dev);
        ctx->usb_host.dev = NULL;
    }

    /* Exit libusb context */
    if (ctx->usb_host.ctx) {
        libusb_exit(ctx->usb_host.ctx);
        ctx->usb_host.ctx = NULL;
    }

    PS2000_LOG_INF("Device closed");
}

/**
 * @brief Perform a USB control transfer.
 *
 * Wrapper around libusb_control_transfer with timeout and
 * transfer length verification.
 *
 * @param ctx      Device context
 * @param req_type Request type (direction | type | recipient)
 * @param req      Request code
 * @param val      wValue field
 * @param idx      wIndex field
 * @param data     Data buffer (may be NULL for zero-length transfers)
 * @param len      Expected transfer length
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_usb_control_transfer(ps2000_context_t *ctx,
                                             uint8_t req_type, uint8_t req,
                                             uint16_t val, uint16_t idx,
                                             uint8_t *data, uint16_t len) {
    if (!ctx || !ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    /* FIX #9: Zero-length transfers are valid (command-only, no data phase) */
    if (len == 0 && data == NULL) {
        int transferred = libusb_control_transfer(
            ctx->usb_host.handle, req_type, req, val, idx,
            NULL, 0, PS2000_USB_TIMEOUT_MS);
        return (transferred >= 0) ? PS2000_OK : _ps2000_usb_map_error(transferred);
    }

    int transferred = libusb_control_transfer(
        ctx->usb_host.handle,
        req_type,
        req,
        val,
        idx,
        data,
        len,
        PS2000_USB_TIMEOUT_MS
    );

    if (transferred < 0) {
        PS2000_LOG_DBG("Control transfer failed: req=0x%02X val=0x%04X "
                       "idx=0x%04X err=%d", req, val, idx, transferred);
        return _ps2000_usb_map_error(transferred);
    }
    if ((uint16_t)transferred != len) {
        PS2000_LOG_WRN("Short control transfer: expected %u, got %d",
                       len, transferred);
        return PS2000_ERR_USB;
    }
    return PS2000_OK;
}

/**
 * @brief Perform a USB bulk transfer.
 *
 * Wrapper around libusb_bulk_transfer with timeout and
 * transfer length verification.
 *
 * @param ctx  Device context
 * @param ep   Endpoint address (IN or OUT)
 * @param data Data buffer
 * @param len  Transfer length in bytes
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_usb_bulk_transfer(ps2000_context_t *ctx, uint8_t ep,
                                          const uint8_t *data, uint32_t len) {
    if (!ctx || !ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;
    if (!data || len == 0) return PS2000_ERR_INVALID_PARAM;

    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle,
        ep,
        (uint8_t*)data,
        (int)len,
        &transferred,
        PS2000_USB_TIMEOUT_MS
    );

    if (ret != LIBUSB_SUCCESS) {
        PS2000_LOG_DBG("Bulk transfer failed: ep=0x%02X len=%u err=%d",
                       ep, len, ret);
        return _ps2000_usb_map_error(ret);
    }
    if ((uint32_t)transferred != len) {
        PS2000_LOG_WRN("Short bulk transfer: expected %u, got %d",
                       len, transferred);
        return PS2000_ERR_USB;
    }
    return PS2000_OK;
}