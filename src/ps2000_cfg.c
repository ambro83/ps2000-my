#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/*                       CONFIGURATION CONSTANTS                              */
/* ========================================================================== */
#define PS2000_CFG_EEPROM_SIZE        256U
#define PS2000_CFG_CALIB_OFFSET       0x200U
#define PS2000_CFG_OFFSET_SCALE       1.0

/* EEPROM header structure (reconstructed from sub_2DDF0) */
#pragma pack(push, 1)
typedef struct _ps2000_eeprom_header {
    uint32_t magic;           /* 0x5053324B ('PS2K') */
    uint16_t version;         /* Format version */
    uint16_t checksum;        /* CRC-16 checksum */
    uint8_t  serial_num[16];  /* Device serial number */
    uint8_t  reserved[32];    /* Reserved */
} ps2000_eeprom_header_t;

typedef struct _ps2000_calib_entry {
    int16_t  offset_raw;      /* Raw offset value (signed) */
    uint32_t gain_raw;        /* Raw gain value (16.16 fixed-point) */
    uint8_t  range_idx;       /* Range index */
    uint8_t  channel;         /* Channel number */
} ps2000_calib_entry_t;
#pragma pack(pop)

/* ========================================================================== */
/*                     INTERNAL CONFIG HELPERS                                */
/* ========================================================================== */

/**
 * @brief Calculate EEPROM checksum using CRC-16.
 *
 * FIX #6: Reconstructed from sub_27025 / sub_27050.
 * Uses CRC-16-CCITT with polynomial 0x1021 and initial value 0xFFFF.
 * This is NOT a simple byte sum.
 *
 * @param data Data buffer to checksum
 * @param len  Length of data in bytes
 * @return 16-bit CRC value
 */
static uint16_t _ps2000_cfg_calc_checksum(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Parse calibration table from EEPROM.
 *
 * Reconstructed from sub_2DDF0+4F7 / sub_2E2A5.
 * Validates magic number and checksum, then extracts
 * calibration data for each channel.
 *
 * FIX #10: gain_raw is 16.16 fixed-point format (divide by 65536.0).
 * FIX #8: offset_raw is stored as-is (raw value). Division by 256.0 happens at use site if needed,
 *         but here we store the raw integer part effectively.
 */
static ps2000_status_t _ps2000_cfg_parse_calib_table(ps2000_context_t *ctx,
                                                     const uint8_t *eeprom_data,
                                                     uint32_t eeprom_len) {
    if (!ctx || !eeprom_data) return PS2000_ERR_INVALID_PARAM;

    /* Check EEPROM size */
    if (eeprom_len < sizeof(ps2000_eeprom_header_t)) {
        PS2000_LOG_ERR("EEPROM too small: %u < %zu",
                       eeprom_len, sizeof(ps2000_eeprom_header_t));
        return PS2000_ERR_CALIBRATION;
    }

    const ps2000_eeprom_header_t *header =
        (const ps2000_eeprom_header_t*)eeprom_data;

    /* Verify magic number */
    if (header->magic != PS2000_EEPROM_MAGIC) {
        PS2000_LOG_ERR("Invalid EEPROM magic: 0x%08X (expected 0x%08X)",
                       header->magic, PS2000_EEPROM_MAGIC);
        return PS2000_ERR_CALIBRATION;
    }

    /* FIX #6: Verify CRC-16 checksum (computed over all bytes except last 2) */
    uint16_t calc_cs = _ps2000_cfg_calc_checksum(eeprom_data, eeprom_len - 2);
    if (calc_cs != header->checksum) {
        PS2000_LOG_ERR("EEPROM CRC-16 mismatch: calc=0x%04X stored=0x%04X",
                       calc_cs, header->checksum);
        return PS2000_ERR_CALIBRATION;
    }

    /* Apply calibration for each channel */
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        uint32_t cal_offset = sizeof(ps2000_eeprom_header_t) +
                              (ch * sizeof(ps2000_calib_entry_t));
        if (cal_offset + sizeof(ps2000_calib_entry_t) > eeprom_len) {
            PS2000_LOG_WRN("EEPROM truncated at channel %u", ch);
            break;
        }

        const ps2000_calib_entry_t *entry =
            (const ps2000_calib_entry_t*)(eeprom_data + cal_offset);

        /*
         * FIX #10: Convert raw values using correct interpretation.
         *
         * gain_raw is 16.16 fixed-point: actual gain = gain_raw / 65536.0
         * FIX #8: offset_raw is kept as raw value. We cast it to int16_t for storage.
         *         The division by 256.0 was previously done here AND in apply/stream,
         *         causing double division. Now we store the raw value.
         */
        double gain = (double)entry->gain_raw / 65536.0;
        double offset = (double)entry->offset_raw; /* Keep raw, divide at use if needed */

        /* Protect against zero gain */
        if (gain == 0.0) {
            gain = 1.0;
        }

        ctx->info.cal_gain[ch] = gain;
        ctx->info.cal_offset[ch] = (int16_t)offset; /* Store raw value */

        PS2000_LOG_DBG("Channel %u calibration: offset_raw=%d gain=%.6f",
                       ch, entry->offset_raw, gain);
    }

    ctx->info.eeprom_valid = 1;
    return PS2000_OK;
}

/* ========================================================================== */
/*                      PUBLIC CONFIG API                                     */
/* ========================================================================== */

/**
 * @brief Load and parse device EEPROM.
 *
 * Reconstructs ps2000_open_unit / sub_2DDF0.
 * FIX #4: Uses IO command mechanism instead of incorrect control transfer.
 *
 * @param ctx Device context
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_cfg_load_eeprom(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    uint8_t eeprom_buf[PS2000_CFG_EEPROM_SIZE];
    memset(eeprom_buf, 0, sizeof(eeprom_buf));

    /* Use IO command mechanism (consistent with _ps2000_hw_io_cmd) */
    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_READ_EEPROM, 0, sizeof(eeprom_buf));
    if (status != PS2000_OK) {
        PS2000_LOG_ERR("Failed to issue EEPROM read command: %d", status);
        return status;
    }

    usleep(1000);

    /* Read response via bulk IN */
    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle,
        PS2000_USB_EP_IN,
        eeprom_buf, sizeof(eeprom_buf),
        &transferred,
        PS2000_USB_TIMEOUT_MS
    );
    if (ret != LIBUSB_SUCCESS || transferred != (int)sizeof(eeprom_buf)) {
        PS2000_LOG_ERR("EEPROM bulk read failed");
        return PS2000_ERR_USB;
    }

    /* Parse and apply calibration */
    status = _ps2000_cfg_parse_calib_table(ctx, eeprom_buf, sizeof(eeprom_buf));
    if (status == PS2000_OK) {
        ps2000_eeprom_header_t *hdr = (ps2000_eeprom_header_t*)eeprom_buf;
        PS2000_LOG_INF("EEPROM loaded: ver=%u, crc=%04X, serial=%s",
                       hdr->version, hdr->checksum, hdr->serial_num);
    } else {
        PS2000_LOG_ERR("Failed to parse EEPROM: %d", status);
    }
    return status;
}

/**
 * @brief Apply calibration to raw data array.
 *
 * Reconstructs vectorized loop from sub_28A19 / sub_28CAA.
 * Formula: result = (raw / 16384.0 * gain + offset) * 32768.0
 *
 * FIX #2: Removed double division of offset. offset is now used directly from ctx->info.cal_offset.
 *
 * @param ctx         Device context
 * @param raw_data    Raw data array (modified in place)
 * @param num_samples Number of samples to process
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_cfg_apply_calibration(ps2000_context_t *ctx,
                                              int16_t *raw_data,
                                              uint32_t num_samples) {
    if (!ctx || !raw_data) return PS2000_ERR_INVALID_PARAM;
    if (num_samples == 0) return PS2000_OK;

    /* Use channel 0 calibration by default */
    double gain = ctx->info.cal_gain[0];
    /* FIX #2: Use offset directly without dividing by 256.0 again */
    double offset = (double)ctx->info.cal_offset[0];

    /* Apply calibration to each sample using correct formula */
    for (uint32_t i = 0; i < num_samples; i++) {
        /* Normalize to [-1.0, +1.0] */
        double normalized = (double)raw_data[i] / PS2000_ADC_FULL_SCALE;
        /* Apply gain and offset */
        double calibrated = (normalized * gain) + offset;
        /* Scale back to int16 range */
        double scaled = calibrated * PS2000_ADC_INT16_SCALE;
        /* Round and clamp */
        int32_t result = (int32_t)_ps2000_math_round_fast(scaled);
        if (result > 32767) result = 32767;
        if (result < -32768) result = -32768;
        raw_data[i] = (int16_t)result;
    }
    return PS2000_OK;
}

/**
 * @brief Get device information.
 *
 * Wrapper around GET_INFO command.
 * Returns serial number and hardware version.
 *
 * FIX #12: Uses proper IO command sequence instead of incorrect control transfer.
 *
 * @param ctx        Device context
 * @param serial_buf Buffer for serial number string
 * @param buf_len    Length of serial buffer
 * @param hw_version Pointer to store hardware version
 * @return PS2000_OK on success, error code otherwise
 */
ps2000_status_t _ps2000_cfg_get_device_info(ps2000_context_t *ctx,
                                            char *serial_buf, uint32_t buf_len,
                                            uint32_t *hw_version) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;
    if (!ctx->usb_host.handle) return PS2000_ERR_NOT_OPENED;

    uint8_t info_buf[64];

    /* Use proper IO command sequence */
    ps2000_status_t status = _ps2000_hw_io_cmd(
        ctx, PS2000_IO_CMD_READ_EEPROM, 1, sizeof(info_buf));
    if (status != PS2000_OK) {
        return status;
    }
    usleep(1000);
    int transferred = 0;
    int ret = libusb_bulk_transfer(
        ctx->usb_host.handle, PS2000_USB_EP_IN,
        info_buf, sizeof(info_buf),
        &transferred, PS2000_USB_TIMEOUT_MS);
    if (ret != LIBUSB_SUCCESS) {
        return PS2000_ERR_USB;
    }

    /* Parse response */
    if (serial_buf && buf_len > 0) {
        uint32_t copy_len = (buf_len - 1 < 16) ? (buf_len - 1) : 16;
        memcpy(serial_buf, info_buf, copy_len);
        serial_buf[copy_len] = '\0';
    }
    if (hw_version) {
        *hw_version = (uint32_t)info_buf[16] | ((uint32_t)info_buf[17] << 8);
    }

    PS2000_LOG_INF("Device info: serial=%s hw_ver=%u",
                   serial_buf ? serial_buf : "N/A",
                   hw_version ? *hw_version : 0);
    return PS2000_OK;
}