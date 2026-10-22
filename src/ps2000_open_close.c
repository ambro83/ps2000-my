/****************************************************************************
* ps2000_open_close.c — Реализация ps2000_open_unit и ps2000_close_unit
*
* Это основные точки входа для работы с устройством.
****************************************************************************/
#include "ps2000_internal.h"
#include "ps2000.h"
#include <string.h>

extern ps2000_global_state_t g_ps2000_state;

/* Placeholder for embedded firmware data if needed */
extern const uint8_t _ps2000_fw_data[];
extern const uint32_t _ps2000_fw_data_len;

/****************************************************************************
* ps2000_open_unit
*
* Opens the first available PS2000 device.
* Returns handle (1-127) on success, 0 on failure.
****************************************************************************/
int16_t ps2000_open_unit(void) {
    /* Initialize global state if needed */
    ps2000_status_t status = _ps2000_init_global_state();
    if (status != PS2000_OK) return 0;

    /* Allocate new context */
    ps2000_context_t *ctx = NULL;
    status = _ps2000_init_context(&ctx);
    if (status != PS2000_OK || !ctx) {
        PS2000_LOG_ERR("Failed to allocate context: %d", status);
        return 0;
    }

    /* Open USB device (serial=0 means first available) */
    status = _ps2000_usb_open_device(ctx, 0);
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("Failed to open USB device: %d", status);
        _ps2000_init_destroy_context(ctx);
        return 0;
    }

    /* Load and parse EEPROM (calibration data) */
    status = _ps2000_cfg_load_eeprom(ctx);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("EEPROM load failed: %d (continuing with defaults)", status);
    }

    /* Initialize hardware sequence */
    status = _ps2000_hw_init_sequence(ctx);
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("Hardware init failed: %d", status);
        _ps2000_usb_close_device(ctx);
        _ps2000_init_destroy_context(ctx);
        return 0;
    }

    /* FIX #4: Load firmware if FPGA is not already loaded */
    if (!ctx->info.fpga_loaded) {
        PS2000_LOG_INF("Loading firmware...");
        status = _ps2000_fw_load(ctx, _ps2000_fw_data, _ps2000_fw_data_len);
        if (status != PS2000_OK) {
            PS2000_LOG_ERR("Firmware load failed: %d", status);
            _ps2000_usb_close_device(ctx);
            _ps2000_init_destroy_context(ctx);
            return 0;
        }
        /* Re-initialize hardware after firmware load if necessary */
        status = _ps2000_hw_init_sequence(ctx);
        if (status != PS2000_OK) {
             PS2000_LOG_ERR("Post-firmware HW init failed: %d", status);
             _ps2000_usb_close_device(ctx);
             _ps2000_init_destroy_context(ctx);
             return 0;
        }
    }

    /* Register context in global array */
    int slot = _ps2000_core_register_context(ctx);
    if (slot < 0) {
        PS2000_LOG_ERR("No available context slots");
        _ps2000_hw_cleanup(ctx);
        _ps2000_usb_close_device(ctx);
        _ps2000_init_destroy_context(ctx);
        return 0;
    }

    /* Return 1-based handle */
    int16_t handle = (int16_t)(slot + PS2000_FIRST_USB);
    PS2000_LOG_INF("Device opened: handle=%d", handle);
    return handle;
}

/****************************************************************************
* ps2000_close_unit
*
* Closes a PS2000 device and frees all resources.
****************************************************************************/
int16_t ps2000_close_unit(int16_t handle) {
    if (handle < PS2000_FIRST_USB || handle > PS2000_LAST_USB) return 0;
    
    uint32_t slot = (uint32_t)(handle - PS2000_FIRST_USB);
    ps2000_context_t *ctx = g_ps2000_state.contexts[slot];
    if (!ctx) return 0;

    PS2000_LOG_INF("Closing device: handle=%d", handle);

    /* Unregister from global array */
    _ps2000_core_unregister_context(ctx);

    /* Close USB device */
    _ps2000_usb_close_device(ctx);

    /* Clean up hardware state */
    _ps2000_hw_cleanup(ctx);

    /* Clean up firmware subsystem */
    _ps2000_fw_cleanup(ctx);

    /* Destroy and free context */
    _ps2000_init_destroy_context(ctx);

    return 1;
}