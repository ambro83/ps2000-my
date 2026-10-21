#include "ps2000_internal.h"
#include <string.h>
#include <time.h>

/* ========================================================================== */
/*                     INIT SCRIPT PARSER                                     */
/* ========================================================================== */

#define INIT_OP_NOP             0x00U
#define INIT_OP_WRITE_REG       0x01U
#define INIT_OP_WRITE_XDATA     0x02U
#define INIT_OP_ADC_SPI         0x03U
#define INIT_OP_GPIO            0x04U
#define INIT_OP_DELAY_MS        0x05U
#define INIT_OP_VERIFY_REG      0x09U
#define INIT_OP_END             0xFFU

/**
 * @brief Parse and execute initialization script from binary blob.
 *
 * FIX #13: _ps2000_hw_write_reg now takes uint16_t reg parameter.
 */
ps2000_status_t _ps2000_init_tbl_execute(ps2000_context_t *ctx,
                                         const uint8_t *script,
                                         uint32_t script_len) {
    if (!ctx || !script || script_len == 0) return PS2000_ERR_INVALID_PARAM;

    PS2000_LOG_INF("Executing init script (%u bytes)...", script_len);

    uint32_t pos = 0;
    uint32_t cmd_count = 0;

    while (pos < script_len) {
        uint8_t opcode = script[pos++];

        if (opcode == INIT_OP_END) {
            PS2000_LOG_INF("Init script completed: %u commands", cmd_count);
            return PS2000_OK;
        }

        ps2000_status_t st = PS2000_OK;

        switch (opcode) {
            case INIT_OP_NOP:
                break;

            case INIT_OP_WRITE_REG: {
                if (pos + 3 > script_len) goto truncated;
                uint16_t reg = (uint16_t)script[pos];  /* FIX #13: uint16_t */
                uint16_t val = (uint16_t)script[pos+1] | ((uint16_t)script[pos+2] << 8);
                st = _ps2000_hw_write_reg(ctx, reg, val);
                pos += 3;
                break;
            }

            case INIT_OP_ADC_SPI: {
                if (pos + 3 > script_len) goto truncated;
                uint8_t ch = script[pos];
                uint16_t val = (uint16_t)script[pos+1] | ((uint16_t)script[pos+2] << 8);
                st = _ps2000_hw_adc_spi_write(ctx, ch, val);
                pos += 3;
                break;
            }

            case INIT_OP_DELAY_MS: {
                if (pos + 2 > script_len) goto truncated;
                uint16_t ms = (uint16_t)script[pos] | ((uint16_t)script[pos+1] << 8);
                struct timespec ts = {.tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L};
                nanosleep(&ts, NULL);
                pos += 2;
                break;
            }

            default:
                PS2000_LOG_WRN("Unknown init opcode: 0x%02X at offset %u", opcode, pos - 1);
                return PS2000_ERR_INVALID_PARAM;
        }

        if (st != PS2000_OK) {
            PS2000_LOG_ERR("Init cmd 0x%02X failed: %d", opcode, st);
            return st;
        }

        cmd_count++;
    }

    PS2000_LOG_WRN("Init script ended without END marker");
    return PS2000_OK;

truncated:
    PS2000_LOG_ERR("Init script truncated at offset %u", pos);
    return PS2000_ERR_INVALID_PARAM;
}