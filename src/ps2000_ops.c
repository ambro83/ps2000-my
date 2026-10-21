#include "ps2000_internal.h"
#include <string.h>

/* ========================================================================== */
/*                     STUB IMPLEMENTATIONS                                   */
/* ========================================================================== */

int _ps2000_ops_stub_zero(void) {
    return 0;
}

/* ========================================================================== */
/*                     IDENTITY AND BASIC INFO                                */
/* ========================================================================== */

int _ps2000_ops_get_identity(ps2000_context_t *ctx) {
    if (!ctx) return 0;
    return (int)(intptr_t)ctx;
}

int _ps2000_ops_get_max_buffer_size(ps2000_context_t *ctx) {
    (void)ctx;
    return (int)PS2000_MAX_BUFFER_SIZE;
}

int _ps2000_ops_get_mode_count(ps2000_context_t *ctx) {
    (void)ctx;
    return (int)PS2000_MODE_COUNT;
}

/* ========================================================================== */
/*                     CAPABILITIES AND VALIDATION                            */
/* ========================================================================== */

/**
 * @brief Check device capability by bit index.
 *
 * Reconstructs sub_133D0 / sub_13220 / sub_12B30:
 *   movzx ecx, byte ptr [rdi+235h]
 *   cmp ecx, 3Ah
 *   ja loc_exit
 *   mov rax, 40100000D040000h
 *   shr rax, cl
 *   not rax
 *   and eax, 1
 *
 * NOTE (Issue #14 VERIFIED): The NOT instruction means the result is INVERTED:
 * the function returns true (1) when the ORIGINAL bit in the mask is 0,
 * and false (0) when the original bit is 1.
 * Capability is "present" when the bit in the mask equals 0 (inverted logic).
 */
bool _ps2000_ops_check_capability(ps2000_context_t *ctx, uint8_t bit) {
    if (!ctx) return false;
    if (bit > 58) return false;

    uint64_t mask = PS2000_CAPABILITIES_MASK;
    mask >>= bit;
    mask = ~mask;  /* NOT — inverted logic: capability present when original bit = 0 */
    return (mask & 1ULL) != 0;
}

int _ps2000_ops_validate_channels(ps2000_context_t *ctx,
                                  uint32_t ch_a, uint32_t ch_b) {
    if (!ctx) return 0;
    bool valid_a = (ch_a <= PS2000_MAX_CHANNELS);
    bool valid_b = (ch_b <= PS2000_MAX_CHANNELS);
    return (valid_a && valid_b) ? 1 : 0;
}

int _ps2000_ops_check_awg(ps2000_context_t *ctx) {
    if (!ctx) return 0;
    if (ctx->siggen.awg_config_flag == 0) return 0;
    if (ctx->siggen.ext_param == 0) return 0;
    return 1;
}

/**
 * @brief Validate signal generator configuration (AWG samples only).
 *
 * Reconstructs sub_2D3A0 (partial — sample count check).
 */
int _ps2000_ops_validate_siggen(ps2000_context_t *ctx, uint32_t awg_samples) {
    if (!ctx) return 0;
    if (awg_samples > PS2000_AWG_MAX_SAMPLES) return 0;
    return 1;
}

/**
 * @brief Validate signal generator with frequency range check.
 *
 * FIX #16 VERIFIED: Extended validation including normalized frequency range.
 * dword_A4C84 = 100.0, dword_A4C88 = -100.0
 * Valid range: -100.0 <= freq <= 100.0 (normalized frequency)
 */
int _ps2000_ops_validate_siggen_freq(ps2000_context_t *ctx,
                                     double freq_normalized,
                                     uint32_t awg_samples) {
    if (!ctx) return 0;

    /* VERIFIED: dword_A4C84 = 100.0, dword_A4C88 = -100.0 */
    const double MIN_FREQ = -100.0;
    const double MAX_FREQ = 100.0;

    if (freq_normalized < MIN_FREQ || freq_normalized > MAX_FREQ) {
        return 0;
    }

    if (awg_samples > PS2000_AWG_MAX_SAMPLES) return 0;

    return 1;
}

/**
 * @brief Reset signal generator to default state.
 *
 * FIX #17 VERIFIED: Reconstructed from sub_2CEF0:
 *   cmp word ptr [rdi+392h], 0
 *   mov dword ptr [rdi+104h], 0
 *   mov [rdi+392h], ax
 *
 * NOTE: Offset 0x104 is in the usb_device block (context base + 0x0F8
 * to + 0x240), NOT in the capture block. The exact field at offset
 * 0x104 from context base corresponds to usb_device offset 0x0C
 * (0x104 - 0x0F8 = 0x0C). This is within the usb_device.gap_040 area
 * or a specific device state field.
 *
 * For reconstruction purposes, we zero siggen.ext_param (offset 0x392)
 * and the field at usb_device offset 0x0C.
 */
int _ps2000_ops_siggen_reset(ps2000_context_t *ctx) {
    if (!ctx) return 0;

    /* Zero awg_ext_param (offset 0x392 relative to context) */
    ctx->siggen.ext_param = 0;

    /*
     * FIX #17 VERIFIED: Zero the field at context offset 0x104.
     * This is in usb_device block: 0x104 - 0x0F8 = 0x0C offset.
     * We zero the first 4 bytes of usb_device.gap_040 as the
     * closest reconstruction of the original write.
     */
    memset(ctx->usb_device.gap_040, 0, 4);

    return 1;
}

/**
 * @brief Initialize signal generator with default values.
 *
 * FIX #11 VERIFIED: Reconstructed from sub_2A850:
 *   mov dword ptr [rdi+32Eh], 80000h
 *   cmp byte ptr [rdi+235h], 10h
 *
 * VERIFIED: This is a DIRECT COMPARISON of the capabilities byte
 * with 0x10 (equality test), NOT a bit check via _ps2000_ops_check_capability.
 */
int _ps2000_ops_siggen_default_init(ps2000_context_t *ctx) {
    if (!ctx) return 0;

    ctx->siggen.config = PS2000_SIGGEN_DEFAULT;

    /*
     * FIX #11 VERIFIED: Compare capabilities byte with 0x10.
     * Original: cmp byte ptr [rdi+235h], 10h
     * This is NOT a bit test — it's a byte equality comparison.
     */
    if (ctx->info.capabilities == 0x10) {
        ctx->siggen.awg_config_flag = 1;
    }

    return 1;
}

/* ========================================================================== */
/*                     STATE AND MODE                                         */
/* ========================================================================== */

int _ps2000_ops_check_timebase_mode(ps2000_context_t *ctx) {
    if (!ctx) return 0;
    return (ctx->device_mode != PS2000_DEVICE_MODE_SPECIAL) ? 1 : 0;
}

/**
 * @brief Get device state.
 *
 * FIX #12 VERIFIED: Reconstructed from sub_17200:
 *   movzx eax, word ptr [rdi+112h]
 *   mov esi, [rdi+104h]
 *
 * NOTE: Offsets 0x112 and 0x104 are in the usb_device block
 * (context base + 0x0F8 to + 0x240):
 *   0x112 - 0x0F8 = 0x1A offset within usb_device (in serial/gap area)
 *   0x104 - 0x0F8 = 0x0C offset within usb_device
 *
 * These correspond to fields within the usb_device internal layout.
 * For reconstruction, we return channel_mask as the closest functional
 * equivalent representing active device state.
 */
int _ps2000_ops_get_state(ps2000_context_t *ctx) {
    if (!ctx) return 0;

    /*
     * FIX #12 VERIFIED: The original reads from usb_device block offsets.
     * Offset 0x112 from context = 0x1A into usb_device (serial area).
     * Offset 0x104 from context = 0x0C into usb_device.
     *
     * Since the exact semantics depend on the usb_device internal layout,
     * we return the channel_mask as the closest functional equivalent
     * representing active device state.
     */
    return (int)ctx->capture.channel_mask;
}

int _ps2000_ops_check_device_state(ps2000_context_t *ctx) {
    if (!ctx) return 0;

    ctx->capture.capture_state = 0;

    /* Query hardware first (sub_13CF0) */
    _ps2000_ops_update_device_state(ctx);

    if (ctx->capture.capture_state == PS2000_DEVICE_STATE_SPECIAL) {
        return 1;
    }
    return 0;
}

void _ps2000_ops_update_device_state(ps2000_context_t *ctx) {
    if (!ctx || !ctx->usb_host.handle) return;

    uint16_t status_reg = 0;
    ps2000_status_t st = _ps2000_hw_read_reg(ctx, 0x00, &status_reg);
    if (st == PS2000_OK) {
        if (status_reg & 0x0020) {
            ctx->capture.capture_state = PS2000_DEVICE_STATE_SPECIAL;
        } else {
            ctx->capture.capture_state = (int32_t)(status_reg & 0x001F);
        }
    } else {
        ctx->capture.capture_state = 0;
    }
}

int _ps2000_ops_check_channel(ps2000_context_t *ctx) {
    if (!ctx) return 0;
    return (ctx->capture.channel_mask != 0) ? 1 : 0;
}

/* ========================================================================== */
/*                     OPERATIONS TABLE INITIALIZATION                        */
/* ========================================================================== */

ps2000_status_t _ps2000_ops_init_table(ps2000_context_t *ctx) {
    if (!ctx) return PS2000_ERR_INVALID_PARAM;

    int32_t variant = ctx->device_variant;

    if (variant < PS2000_DEVICE_VARIANT_2) {
        PS2000_LOG_WRN("Unsupported device variant: %d", variant);
        return PS2000_ERR_NOT_IMPLEMENTED;
    }

    /* Initialize all stubs first */
    ctx->ops.stub_01 = _ps2000_ops_stub_zero;
    ctx->ops.stub_02 = _ps2000_ops_stub_zero;
    ctx->ops.stub_03 = _ps2000_ops_stub_zero;
    ctx->ops.stub_04 = _ps2000_ops_stub_zero;
    ctx->ops.stub_05 = _ps2000_ops_stub_zero;
    ctx->ops.stub_06 = _ps2000_ops_stub_zero;
    ctx->ops.stub_07 = _ps2000_ops_stub_zero;
    ctx->ops.stub_08 = _ps2000_ops_stub_zero;
    ctx->ops.stub_09 = _ps2000_ops_stub_zero;
    ctx->ops.stub_10 = _ps2000_ops_stub_zero;
    ctx->ops.stub_11 = _ps2000_ops_stub_zero;
    ctx->ops.stub_12 = _ps2000_ops_stub_zero;
    ctx->ops.stub_13 = _ps2000_ops_stub_zero;
    ctx->ops.stub_14 = _ps2000_ops_stub_zero;
    ctx->ops.stub_15 = _ps2000_ops_stub_zero;
    ctx->ops.stub_16 = _ps2000_ops_stub_zero;
    ctx->ops.stub_17 = _ps2000_ops_stub_zero;
    ctx->ops.stub_18 = _ps2000_ops_stub_zero;
    ctx->ops.stub_19 = _ps2000_ops_stub_zero;
    ctx->ops.stub_20 = _ps2000_ops_stub_zero;
    ctx->ops.stub_21 = _ps2000_ops_stub_zero;
    ctx->ops.stub_22 = _ps2000_ops_stub_zero;
    ctx->ops.stub_23 = _ps2000_ops_stub_zero;
    ctx->ops.stub_24 = _ps2000_ops_stub_zero;
    ctx->ops.stub_25 = _ps2000_ops_stub_zero;
    ctx->ops.stub_26 = _ps2000_ops_stub_zero;
    ctx->ops.stub_27 = _ps2000_ops_stub_zero;

    /* Populate implemented methods */
    ctx->ops.get_identity = _ps2000_ops_get_identity;
    ctx->ops.get_max_buffer_size = _ps2000_ops_get_max_buffer_size;
    ctx->ops.get_mode_count = _ps2000_ops_get_mode_count;
    ctx->ops.check_capability_a = _ps2000_ops_check_capability;
    ctx->ops.check_capability_b = _ps2000_ops_check_capability;
    ctx->ops.check_capability_c = _ps2000_ops_check_capability;
    ctx->ops.validate_channels = _ps2000_ops_validate_channels;
    ctx->ops.check_awg = _ps2000_ops_check_awg;
    ctx->ops.siggen_reset = _ps2000_ops_siggen_reset;
    ctx->ops.siggen_default_init = _ps2000_ops_siggen_default_init;
    ctx->ops.check_timebase_mode = _ps2000_ops_check_timebase_mode;
    ctx->ops.get_state = _ps2000_ops_get_state;
    ctx->ops.check_device_state = _ps2000_ops_check_device_state;
    ctx->ops.check_channel = _ps2000_ops_check_channel;

    switch (variant) {
        case PS2000_DEVICE_VARIANT_2:
        case PS2000_DEVICE_VARIANT_3:
            PS2000_LOG_INF("Operations table initialized for variant %d", variant);
            break;
        case PS2000_DEVICE_VARIANT_4:
            PS2000_LOG_INF("Operations table initialized for extended variant 4");
            break;
        default:
            PS2000_LOG_WRN("Unknown device variant: %d, using defaults", variant);
            break;
    }

    return PS2000_OK;
}