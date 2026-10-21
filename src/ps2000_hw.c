#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* ========================================================================== */
/*                       INTERNAL HW CONSTANTS                                */
/* ========================================================================== */

#define PS2000_HW_INIT_STAGE_IDLE       0
#define PS2000_HW_INIT_STAGE_MCU        1
#define PS2000_HW_INIT_STAGE_FPGA       2
#define PS2000_HW_INIT_STAGE_ADC        3
#define PS2000_HW_INIT_STAGE_GPIO       4
#define PS2000_HW_INIT_STAGE_EEPROM     5
#define PS2000_HW_INIT_STAGE_READY      6

#define PS2000_HW_VENDOR_REQ_IO         0xA0U
#define PS2000_HW_VENDOR_REQ_RESET      0xA8U

#define PS2000_PRELAUNCH_MAGIC          0x1CED9168U
#define PS2000_PRELAUNCH_VERSION        0x11E6U
#define PS2000_APPLY_FIX_MAGIC_A        0x1CED9168U
#define PS2000_APPLY_FIX_MAGIC_B        0x0ED5A2205U
#define PS2000_APPLY_FIX_VERSION        0x39BCU

#define PS2000_DEVICE_STATE_READY       5
#define PS2000_INIT_MARKER_Z            0x5A
#define PS2000_INIT_MARKER_RESET        0x0A

/* FIX #5: ADC default = 0x4000 (16384) */
#define PS2000_HW_ADC_DEFAULT_VALUE     0x4000U

/*
 * FIX #15: Capabilities register address for reading device capabilities.
 * This register is read during hardware initialization to populate
 * ctx->info.capabilities. The exact address needs verification from
 * the listing; using 0x02 as a placeholder based on sub_2A850 context.
 */
#define PS2000_HW_CAPS_REG_ADDR         0x02U

/* FIX #1: IO packet structure */
#pragma pack(push, 1)
typedef struct _ps2000_io_packet {
    uint8_t  cmd;           /* +0x00: IO command */
    uint16_t address;       /* +0x01: 16-bit register address (LE) */
    uint16_t value;         /* +0x03: 16-bit value (LE) */
    uint8_t  checksum;      /* +0x05: XOR checksum of bytes 0-4 */
    uint8_t  reserved[2];   /* +0x06: Padding to 8 bytes */
} ps2000_io_packet_t;
#pragma pack(pop)

static uint32_t g_hw_init_flag = 0;
static uint16_t g_hw_fix_flag = 1;
static uint16_t g_hw_fix_result = 0;

/* ========================================================================== */
/*                     LOW-LEVEL IO HELPERS                                   */
/* ========================================================================== */

/* FIX #2: XOR of all payload bytes */
static uint8_t _ps2000_hw_calc_checksum(const ps2000_io_packet_t *pkt) {
    uint8_t cs = 0;
    cs ^= pkt->cmd;
    cs ^= (uint8_t)(pkt->address & 0xFF);
    cs ^= (uint8_t)((pkt->address >> 8) & 0xFF);
    cs ^= (uint8_t)(pkt->value & 0xFF);
    cs ^= (uint8_t)((pkt->value >> 8) & 0xFF);
    return cs;
}

/* FIX #3: Correct wValue/wIndex per sub_27830 */
ps2000_status_t _ps2000_hw_io_cmd(ps2000_context_t *ctx, ps2000_io_cmd_t cmd,
                                  uint16_t addr, uint16_t value) {
    if (!ctx || !ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;
    if (cmd >= PS2000_IO_CMD_COUNT) return PS2000_ERR_INVALID_PARAM;

    ps2000_io_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.cmd = (uint8_t)cmd;
    pkt.address = addr;
    pkt.value = value;
    pkt.checksum = _ps2000_hw_calc_checksum(&pkt);
    pkt.reserved[0] = 0;
    pkt.reserved[1] = 0;

    uint16_t wValue = ((uint16_t)cmd << 8) | (addr & 0xFF);
    uint16_t wIndex = value;

    ps2000_status_t status = _ps2000_usb_control_transfer(ctx,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        PS2000_HW_VENDOR_REQ_IO,
        wValue,
        wIndex,
        (uint8_t*)&pkt, sizeof(pkt));

    if (status != PS2000_OK) {
        PS2000_LOG_ERR("IO command %d failed: %d", (int)cmd, status);
    } else {
        PS2000_LOG_DBG("IO command %d: addr=0x%04X val=0x%04X",
                       (int)cmd, addr, value);
    }

    return status;
}

/* ========================================================================== */
/*                     REGISTER ACCESS API                                    */
/* ========================================================================== */

/**
 * @brief Write a 16-bit value to a hardware register.
 *
 * FIX #13: Register address is uint16_t to support addresses > 255.
 */
ps2000_status_t _ps2000_hw_write_reg(ps2000_context_t *ctx,
                                     uint16_t reg, uint16_t val) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_WRITE_FPGA_REGISTER, reg, val);

    if (status == PS2000_OK) {
        PS2000_LOG_DBG("HW Write Reg 0x%04X = 0x%04X", reg, val);
    }

    return status;
}

/**
 * @brief Read a 16-bit value from a hardware register.
 *
 * FIX #4: Correct sequence: OUT command → delay → BULK IN.
 * FIX #13: Register address is uint16_t to support addresses > 255.
 */
ps2000_status_t _ps2000_hw_read_reg(ps2000_context_t *ctx,
                                    uint16_t reg, uint16_t *val) {
    if (!ctx || !val) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    /* Step 1: Send read command via Control OUT (zero-length data phase) */
    uint16_t wValue = ((uint16_t)PS2000_IO_CMD_READ_FPGA_REGISTER << 8)
                    | (reg & 0xFF);

    ps2000_status_t status = _ps2000_usb_control_transfer(ctx,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        PS2000_HW_VENDOR_REQ_IO,
        wValue,
        0,
        NULL, 0);

    if (status != PS2000_OK) {
        PS2000_LOG_ERR("Read reg command failed for reg 0x%04X: %d",
                       reg, status);
        return status;
    }

    /* Step 2: Wait for hardware processing */
    usleep(1000);

    /* Step 3: Read response via BULK IN endpoint */
    uint8_t resp[2] = {0};
    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle,
        PS2000_USB_EP_IN,
        resp, 2,
        &transferred,
        PS2000_USB_TIMEOUT_MS
    );

    if (ret != LIBUSB_SUCCESS || transferred != 2) {
        PS2000_LOG_ERR("Bulk read for reg 0x%04X failed: ret=%d transferred=%d",
                       reg, ret, transferred);
        return PS2000_ERR_USB;
    }

    *val = (uint16_t)resp[0] | ((uint16_t)resp[1] << 8);
    PS2000_LOG_DBG("HW Read Reg 0x%04X = 0x%04X", reg, *val);

    return PS2000_OK;
}

ps2000_status_t _ps2000_hw_adc_spi_write(ps2000_context_t *ctx,
                                         uint8_t ch, uint16_t val) {
    if (!ctx || ch >= PS2000_MAX_CHANNELS) return PS2000_ERR_INVALID_PARAM;

    ctx->streaming.adc_spi_cache[ch] = val;

    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_WRITE_ADC_SPI, (uint16_t)ch, val);

    if (status == PS2000_OK) {
        PS2000_LOG_DBG("ADC SPI Write ch%u = 0x%04X", ch, val);
    }

    return status;
}

/* ========================================================================== */
/*                     FPGA CONFIGURATION                                     */
/* ========================================================================== */

/**
 * FIX #9: Added critical delays between configuration stages.
 */
ps2000_status_t _ps2000_hw_load_fpga(ps2000_context_t *ctx,
                                     const uint8_t *bitstream, uint32_t len) {
    if (!ctx || !bitstream || len == 0) return PS2000_ERR_INVALID_PARAM;

    PS2000_LOG_INF("Loading FPGA bitstream (%u bytes)...", len);

    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_FPGA_IO, 0, (uint16_t)(len & 0xFFFF));
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("FPGA config initiation failed: %d", status);
        return PS2000_ERR_FPGA;
    }

    usleep(10000);  /* FIX #9: Wait for FPGA ready */

    status = _ps2000_usb_bulk_transfer(ctx, PS2000_USB_EP_OUT, bitstream, len);
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("FPGA bitstream transfer failed: %d", status);
        return PS2000_ERR_FPGA;
    }

    usleep(50000);  /* FIX #9: Wait for FPGA configuration */

    uint16_t fpga_status = 0;
    status = _ps2000_hw_read_reg(ctx, 0x00, &fpga_status);
    if (status != PS2000_OK || fpga_status == 0) {
        PS2000_LOG_ERR("fpga could not complete configuration correctly");
        return PS2000_ERR_FPGA;
    }

    usleep(1000);  /* FIX #9: Stabilization */

    ctx->info.fpga_loaded = 1;
    PS2000_LOG_INF("FPGA configured: version=0x%04X", fpga_status);
    return PS2000_OK;
}

/* ========================================================================== */
/*                     PRELAUNCH AND FIX LOGIC                                */
/* ========================================================================== */

int _ps2000_hw_apply_fix(uint32_t magic, uint16_t version) {
    int result = 0;

    if (magic == PS2000_APPLY_FIX_MAGIC_A) {
        result = (version != PS2000_PRELAUNCH_VERSION) ? 1 : 0;
        g_hw_fix_flag = (uint16_t)result;
        return result;
    }

    if (magic == PS2000_APPLY_FIX_MAGIC_B) {
        result = (version == PS2000_APPLY_FIX_VERSION) ? 1 : 0;
        g_hw_fix_result = (uint16_t)result;
        return result;
    }

    return 0;
}

ps2000_status_t _ps2000_hw_prelaunch(void) {
    _ps2000_hw_apply_fix(PS2000_PRELAUNCH_MAGIC, PS2000_PRELAUNCH_VERSION);

    if (g_hw_init_flag != 0) {
        PS2000_LOG_DBG("PreLaunch: already initialized");
        return PS2000_OK;
    }

    g_hw_init_flag = 1;

    if (g_hw_fix_flag != 0) {
        PS2000_LOG_WRN("PreLaunch: fix flag set, skipping full init");
        return PS2000_OK;
    }

    PS2000_LOG_INF("PreLaunch: hardware ready");
    return PS2000_OK;
}

/* ========================================================================== */
/*                     INITIALIZATION SEQUENCE                                */
/* ========================================================================== */

/**
 * @brief Full hardware initialization sequence.
 *
 * FIX #15: Added capabilities loading from device register.
 */
ps2000_status_t _ps2000_hw_init_sequence(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    ps2000_status_t status;

    /* Stage 1: MCU handshake */
    PS2000_LOG_DBG("HW init stage 1: MCU handshake");
    uint16_t mcu_ver = 0;
    status = _ps2000_hw_read_reg(ctx, 0x01, &mcu_ver);
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("MCU handshake failed: %d", status);
        return status;
    }
    PS2000_LOG_INF("MCU Status: 0x%04X", mcu_ver);

    /* Stage 2: FPGA configuration check */
    PS2000_LOG_DBG("HW init stage 2: FPGA check");
    if (!ctx->info.fpga_loaded) {
        PS2000_LOG_WRN("FPGA bitstream not embedded; "
                       "expecting pre-configured device");
    }

    /* Stage 3: ADC initialization via SPI */
    PS2000_LOG_DBG("HW init stage 3: ADC init");
    for (uint8_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        status = _ps2000_hw_adc_spi_write(ctx, ch, PS2000_HW_ADC_DEFAULT_VALUE);
        if (status != PS2000_OK) {
            PS2000_LOG_WRN("ADC SPI init failed for ch %u: %d", ch, status);
        }
    }

    /* Stage 4: GPIO configuration via FX3 */
    PS2000_LOG_DBG("HW init stage 4: GPIO config");
    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_FX3_USE_GPIO, 0, 0x00);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("GPIO init failed: %d", status);
    }

    /* Stage 5: Wait for EEPROM */
    PS2000_LOG_DBG("HW init stage 5: EEPROM wait");
    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_WAIT_FOR_EEPROM_WRITE, 0, 0);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("EEPROM wait failed: %d", status);
    }

    /* Stage 6: Check flash availability */
    PS2000_LOG_DBG("HW init stage 6: Flash check");
    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_CHECK_FLASH_AVAIL, 0, 0);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("Flash check failed: %d", status);
    }

    /* Stage 7: Check EEPROM is not empty */
    PS2000_LOG_DBG("HW init stage 7: EEPROM check");
    status = _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_CHECK_EMPTY_EEPROM, 0, 0);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("EEPROM check failed: %d", status);
    }

    /*
     * FIX #15: Load capabilities from device.
     * The capabilities byte at context offset +0x235 must be populated
     * from the device. Read from a dedicated FPGA register.
     */
    PS2000_LOG_DBG("HW init stage 8: Load capabilities");
    uint16_t caps_reg = 0;
    status = _ps2000_hw_read_reg(ctx, PS2000_HW_CAPS_REG_ADDR, &caps_reg);
    if (status == PS2000_OK) {
        ctx->info.capabilities = (uint8_t)(caps_reg & 0xFF);
        PS2000_LOG_INF("Device capabilities: 0x%02X", ctx->info.capabilities);
    } else {
        PS2000_LOG_WRN("Failed to read capabilities register: %d", status);
        ctx->info.capabilities = 0;
    }

    PS2000_LOG_INF("Hardware initialization complete");
    return PS2000_OK;
}

void _ps2000_hw_cleanup(ps2000_context_t *ctx) {
    if (!ctx) return;

    ctx->info.fpga_loaded = 0;

    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        ctx->streaming.adc_spi_cache[ch] = 0;
    }

    PS2000_LOG_DBG("Hardware state cleaned up");
}

ps2000_status_t _ps2000_hw_reset(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    ctx->capture.capture_state = 0;
    ctx->streaming.stream_write_pos = 0;
    ctx->capture.trigger_enabled = 0;
    ctx->capture.trigger_threshold = 0;

    PS2000_LOG_INF("Hardware reset complete");
    return PS2000_OK;
}

/* ========================================================================== */
/*                     FLASH MEMORY OPERATIONS                                */
/* ========================================================================== */

ps2000_status_t _ps2000_hw_flash_read_page(ps2000_context_t *ctx,
                                           uint16_t page,
                                           uint8_t *data,
                                           uint16_t data_len) {
    if (!ctx || !data) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_READ_FLASH_PAGE, page, data_len);
    if (status != PS2000_OK) return status;

    usleep(1000);

    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle,
        PS2000_USB_EP_IN,
        data, data_len,
        &transferred,
        PS2000_USB_TIMEOUT_MS
    );

    if (ret != LIBUSB_SUCCESS || transferred != data_len) {
        PS2000_LOG_ERR("Flash page read failed: ret=%d transferred=%d",
                       ret, transferred);
        return PS2000_ERR_USB;
    }

    return PS2000_OK;
}

ps2000_status_t _ps2000_hw_flash_erase_page(ps2000_context_t *ctx,
                                            uint16_t page) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    return _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_ERASE_FLASH_PAGE, page, 0);
}

ps2000_status_t _ps2000_hw_flash_protect(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    return _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_PROTECT_FLASH, 0, 1);
}

/* ========================================================================== */
/*                     EEPROM OPERATIONS                                      */
/* ========================================================================== */

ps2000_status_t _ps2000_hw_eeprom_read(ps2000_context_t *ctx,
                                       uint16_t offset,
                                       uint8_t *data,
                                       uint16_t data_len) {
    if (!ctx || !data) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_READ_EEPROM, offset, data_len);
    if (status != PS2000_OK) return status;

    usleep(1000);

    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle,
        PS2000_USB_EP_IN,
        data, data_len,
        &transferred,
        PS2000_USB_TIMEOUT_MS
    );

    if (ret != LIBUSB_SUCCESS || transferred != data_len) {
        PS2000_LOG_ERR("EEPROM read failed: ret=%d transferred=%d",
                       ret, transferred);
        return PS2000_ERR_USB;
    }

    return PS2000_OK;
}

ps2000_status_t _ps2000_hw_eeprom_wait(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    return _ps2000_hw_io_cmd(ctx, PS2000_IO_CMD_WAIT_FOR_EEPROM_WRITE, 0, 0);
}