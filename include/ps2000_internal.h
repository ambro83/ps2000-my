#ifndef PS2000_INTERNAL_H
#define PS2000_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           ERROR CODES                                      */
/* ========================================================================== */
typedef int32_t ps2000_status_t;

#define PS2000_OK                     ((ps2000_status_t) 0)
#define PS2000_ERR_INVALID_PARAM      ((ps2000_status_t)-1)
#define PS2000_ERR_MEMORY             ((ps2000_status_t)-2)
#define PS2000_ERR_NOT_FOUND          ((ps2000_status_t)-3)
#define PS2000_ERR_USB                ((ps2000_status_t)-4)
#define PS2000_ERR_NOT_OPENED         ((ps2000_status_t)-5)
#define PS2000_ERR_BUSY               ((ps2000_status_t)-6)
#define PS2000_ERR_TIMEOUT            ((ps2000_status_t)-7)
#define PS2000_ERR_INDEX_MODE         ((ps2000_status_t)-8)
#define PS2000_ERR_SIGGEN_CFG         ((ps2000_status_t)-9)
#define PS2000_ERR_FPGA               ((ps2000_status_t)-10)
#define PS2000_ERR_CALIBRATION        ((ps2000_status_t)-11)
#define PS2000_ERR_STREAMING          ((ps2000_status_t)-12)
#define PS2000_ERR_TRIGGER            ((ps2000_status_t)-13)
#define PS2000_ERR_FIRMWARE           ((ps2000_status_t)-14)
#define PS2000_ERR_NOT_IMPLEMENTED    ((ps2000_status_t)-15)
#define PS2000_ERR_COMMUNICATION      ((ps2000_status_t)-16)

/* ========================================================================== */
/*                     INTERNAL CONSTANTS & LIMITS                            */
/* ========================================================================== */
#define PS2000_MAX_UNITS              127U
#define PS2000_MAX_CHANNELS           4U
#define PS2000_MAX_TRIGGERS           2U
#define PS2000_USB_TIMEOUT_MS         1000U
#define PS2000_EEPROM_MAGIC           0x5053324BU
#define PS2000_DRIVER_VERSION         "3.0.150.6187"

/* FIX #6: Added missing constants */
#define PS2000_MAX_TIMEBASE           19U
#define PS2000_OSC_MAX_SAMPLES        0xFFFEU

/* USB endpoint addresses (from sub_4E3D9) */
#define PS2000_USB_EP_IN              0x81U
#define PS2000_USB_EP_OUT             0x01U

/* Device variants (from sub_237C0 prologue: cmp eax, 4) */
#define PS2000_DEVICE_VARIANT_2       2U
#define PS2000_DEVICE_VARIANT_3       3U
#define PS2000_DEVICE_VARIANT_4       4U

/* ========================================================================== */
/*            RECONSTRUCTED MATH CONSTANTS (semantic names)                   */
/* ========================================================================== */
/* --- Fast rounding via 2^52 bias (sub_203A0, sub_3AA64, sub_38CB8) --- */
#define PS2000_FAST_ROUND_MAGIC       4503599627370496.0  /* 2^52 */
#define PS2000_DOUBLE_ABS_MASK        0x7FFFFFFFFFFFFFFFULL
#define PS2000_DOUBLE_SIGN_MASK       0x8000000000000000ULL

/* --- ADC scaling (sub_F9E4: cvtsi2sd -> mulsd -> cvttsd2si) --- */
#define PS2000_ADC_LSB_WEIGHT         1.0         /* Base ADC scale factor */
#define PS2000_ADC_FULL_SCALE         16384.0     /* 2^14, 14-bit ADC */
#define PS2000_ADC_INT16_SCALE        32768.0     /* 2^15, int16 range */

/* --- Timebase calculation (sub_10BF0, sub_2AC11) --- */
#define PS2000_TIMEBASE_ROUND_BIAS    0.5
#define PS2000_TIMEBASE_NS_PER_SEC    1000000000.0
#define PS2000_TIMEBASE_MAX_INTERVAL  2147483647.0  /* INT32_MAX */
#define PS2000_TIMEBASE_LOG_BASE      10.0
#define PS2000_TIMEBASE_UNIT_STEP     1.0
#define PS2000_TIMEBASE_PERCENT_SCALE 100.0
#define PS2000_TIMEBASE_FINE_STEP     0.00015707963267948966 /* ~pi/20000 */

/* --- Signal generator / AWG (sub_28450, sub_2D3A0) --- */
#define PS2000_AWG_MAX_SAMPLES        0x3FFFFFU   /* from cmp esi, 3FFFFFh */
#define PS2000_SIGGEN_DEFAULT         0x80000U    /* from sub_2A850 */
#define PS2000_DAC_FULL_SCALE         4095.5
#define PS2000_DAC_MID_SCALE          2047.0
#define PS2000_MATH_PI                3.141592653589793
/* FIX #102: Updated to match DDS reference clock in gen.c (200 MHz) */
#define PS2000_AWG_REF_SAMPLE_RATE    200000000.0  /* 200 MHz (DDS master clock) */
#define PS2000_AWG_AMPLITUDE_SCALE    128.0
#define PS2000_AWG_OFFSET_BIAS        (-16.0)
#define PS2000_AWG_QUARTER_SCALE      0.25
#define PS2000_AWG_MAX_PHASE          2000000.0

/* --- Decimation / streaming --- */
#define PS2000_DECIMATION_DIVISOR     0.0625      /* 1/16 */
#define PS2000_MIN_SAMPLE_INTERVAL_S  5.0e-8      /* 50 ns */

/* --- Frequency calculations --- */
#define PS2000_FREQ_PROBE_DIVIDER     20.0
#define PS2000_FREQ_RANGE_LIMIT       500.0

/* --- Miscellaneous scaling --- */
#define PS2000_EIGHTH_DIVISOR         0.125       /* 1/8 */
#define PS2000_FOUR_CHANNELS          4.0
#define PS2000_INT32_MAX_MASK         0x7FFFFFFF

/* --- Float constants (single precision) --- */
#define PS2000_FLOAT_UNIT             1.0f
#define PS2000_FLOAT_MS_SCALE         1000.0f
#define PS2000_FLOAT_NS_SCALE         1000000.0f
#define PS2000_TRIG_MAX_LIMIT_F       100.0f
#define PS2000_TRIG_MIN_LIMIT_F       (-100.0f)
#define PS2000_FLOAT_NEG_UNIT         (-1.0f)
#define PS2000_FLOAT_PROBE_X10_X11    1.1f

/* --- Capabilities bitmap (from sub_133D0) --- */
#define PS2000_CAPABILITIES_MASK      0x40100000D040000ULL

/* --- Operation dispatcher constants (from sub_237C0 callees) --- */
#define PS2000_MAX_BUFFER_SIZE        0xA00U      /* from sub_22A00 */
#define PS2000_MODE_COUNT             7U          /* from sub_220A0 */
#define PS2000_DEVICE_STATE_SPECIAL   5           /* from sub_14610 */
#define PS2000_DEVICE_MODE_SPECIAL    5           /* from sub_173F0 */

/* ========================================================================== */
/*                     IO COMMAND ENUM (from RTTI strings)                    */
/* ========================================================================== */
typedef enum {
    PS2000_IO_CMD_READ_XDATA = 0,
    PS2000_IO_CMD_WRITE_XDATA,
    PS2000_IO_CMD_READ_EXTENDED_REGISTER,
    PS2000_IO_CMD_WRITE_EXTENDED_REGISTER,
    PS2000_IO_CMD_READ_I2C,
    PS2000_IO_CMD_READ_I2C_WITH_STATUS,
    PS2000_IO_CMD_WRITE_I2C,
    PS2000_IO_CMD_WRITE_SPI,
    PS2000_IO_CMD_WRITE_ADC_SPI,
    PS2000_IO_CMD_READ_WRITE_SPI,
    PS2000_IO_CMD_WRITE_I2C_VIA_SPI,
    PS2000_IO_CMD_READ_I2C_VIA_SPI,
    PS2000_IO_CMD_READ_WRITE_ADC_SPI,
    PS2000_IO_CMD_WAIT_FOR_EEPROM_WRITE,
    PS2000_IO_CMD_WAIT_FOR_FX3_EEPROM_WRITE,
    PS2000_IO_CMD_READ_FPGA_REGISTER,
    PS2000_IO_CMD_WRITE_FPGA_REGISTER,
    PS2000_IO_CMD_READ_WRITE_FPGA_REGISTER,
    PS2000_IO_CMD_READ_FIFO,
    PS2000_IO_CMD_ABORT_READ_FIFO,
    PS2000_IO_CMD_WRITE_FIFO,
    PS2000_IO_CMD_SET_AUTOIN_LEN,
    PS2000_IO_CMD_DELAY_MS,
    PS2000_IO_CMD_READ_EEPROM,
    PS2000_IO_CMD_WRITE_EEPROM,
    PS2000_IO_CMD_FX3_USE_GPIO,
    PS2000_IO_CMD_FX3_UNUSE_GPIO,
    PS2000_IO_CMD_FX3_IS_GPIF_DONE,
    PS2000_IO_CMD_FX3_GPIO_PWM,
    PS2000_IO_CMD_USB_WRITE_GPIF_FREQ,
    PS2000_IO_CMD_ERASE_FLASH_PAGE,
    PS2000_IO_CMD_SELECT_CSN_SPI,
    PS2000_IO_CMD_CHECK_EMPTY_EEPROM,
    PS2000_IO_CMD_READ_FLASH_PAGE,
    PS2000_IO_CMD_PROTECT_FLASH,
    PS2000_IO_CMD_CHECK_FLASH_AVAIL,
    PS2000_IO_CMD_WRITE_FLASH_PAGE,
    PS2000_IO_CMD_PACKET,
    PS2000_IO_CMD_FPGA_IO,
    PS2000_IO_CMD_COUNT
} ps2000_io_cmd_t;

/* ========================================================================== */
/*                     FUNCTION POINTER TYPES FOR OPS TABLE                   */
/* ========================================================================== */
/* Forward declaration */
typedef struct _ps2000_context ps2000_context_t;

/* FIX #1: Added new function pointer types for Advanced Trigger and Streaming */
typedef int (*ps2000_op_adv_trig_props_t)(ps2000_context_t *ctx, void *props, int16_t n_props, int32_t auto_ms);
typedef int (*ps2000_op_adv_trig_conds_t)(ps2000_context_t *ctx, void *conds, int16_t n_conds);
typedef int (*ps2000_op_adv_trig_dirs_t)(ps2000_context_t *ctx, uint16_t dirs);
typedef int (*ps2000_op_pwq_t)(ps2000_context_t *ctx, void *conds, int16_t n_conds, uint16_t dir, uint32_t lower, uint32_t upper, uint32_t type);
typedef int (*ps2000_op_adv_trig_delay_t)(ps2000_context_t *ctx, uint32_t delay, float pre_delay);
typedef int (*ps2000_op_start_streaming_t)(ps2000_context_t *ctx, uint16_t interval_us, uint32_t max_samples, int16_t auto_stop);

typedef int             (*ps2000_op_void_t)(void);
typedef int             (*ps2000_op_ctx_t)(ps2000_context_t *ctx);
typedef int             (*ps2000_op_ctx_u32_t)(ps2000_context_t *ctx, uint32_t val);
typedef int             (*ps2000_op_ctx_u32_u32_t)(ps2000_context_t *ctx,
                                                   uint32_t a, uint32_t b);
typedef int             (*ps2000_op_ctx_u32_u32_u32_t)(ps2000_context_t *ctx,
                                                       uint32_t a, uint32_t b,
                                                       uint32_t c);
typedef int             (*ps2000_op_ctx_i32_t)(ps2000_context_t *ctx, int32_t val);
typedef int             (*ps2000_op_ctx_ptr_t)(ps2000_context_t *ctx, void *ptr);
typedef int             (*ps2000_op_ctx_ptr_u32_t)(ps2000_context_t *ctx,
                                                   void *ptr, uint32_t len);
typedef const char*     (*ps2000_op_get_str_t)(ps2000_context_t *ctx, uint32_t type);
typedef bool            (*ps2000_op_check_t)(ps2000_context_t *ctx, uint8_t bit);
typedef double          (*ps2000_op_calc_dbl_t)(ps2000_context_t *ctx, uint32_t index);

/* ========================================================================== */
/*                     OPERATIONS TABLE (sub_237C0 reconstruction)            */
/* ========================================================================== */
/**
 * @brief Device operations table.
 *
 * Reconstructed from sub_237C0 which populates function pointers
 * based on device variant (context offset +0xAA).
 *
 * FIX #1: Extended with Advanced Trigger and Streaming delegates.
 */
typedef struct _ps2000_operations {
    /* Identity and basic info */
    ps2000_op_ctx_t             get_identity;           /* sub_26F00 */
    ps2000_op_ctx_t             get_max_buffer_size;    /* sub_22A00: returns 0xA00 */
    ps2000_op_ctx_t             get_mode_count;         /* sub_220A0: returns 7 */

    /* Capture and buffering */
    ps2000_op_ctx_u32_u32_t     run_block;              /* sub_2ADC0 */
    ps2000_op_ctx_ptr_u32_t     allocate_buffers;       /* sub_2C350 */
    ps2000_op_ctx_ptr_u32_t     process_data;           /* sub_203A0 */
    ps2000_op_ctx_t             get_capture_status;     /* sub_326B0 */

    /* Timebase */
    ps2000_op_calc_dbl_t        calc_timebase;          /* sub_10BF0 */
    ps2000_op_calc_dbl_t        calc_timebase_alt;      /* sub_11110 */
    ps2000_op_ctx_t             check_timebase_mode;    /* sub_173F0 */

    /* Signal generator */
    ps2000_op_ctx_u32_t         validate_siggen;        /* sub_2D3A0 */
    ps2000_op_ctx_u32_t         siggen_calc;            /* sub_35E40 */
    ps2000_op_ctx_t             siggen_reset;           /* sub_2CEF0 */
    ps2000_op_ctx_t             check_awg;              /* sub_2A830 */
    ps2000_op_ctx_t             siggen_default_init;    /* sub_2A850 */
    ps2000_op_ctx_u32_t         calc_freq;              /* sub_13550 */

    /* Capabilities and validation */
    ps2000_op_check_t           check_capability_a;     /* sub_133D0 */
    ps2000_op_check_t           check_capability_b;     /* sub_13220 */
    ps2000_op_check_t           check_capability_c;     /* sub_12B30 */
    ps2000_op_ctx_u32_u32_t     validate_channels;      /* sub_2D2C0 */
    ps2000_op_ctx_t             validate_config;        /* sub_2D410 */

    /* Trigger */
    ps2000_op_ctx_u32_t         set_trigger;            /* sub_1C310 */
    ps2000_op_ctx_t             check_trigger_mode;     /* sub_17C10 */
    ps2000_op_ctx_t             check_capture_mode;     /* sub_13DB0 */

    /* Information retrieval */
    ps2000_op_get_str_t         get_info_string;        /* sub_14B70 */
    ps2000_op_get_str_t         get_string_param_a;     /* sub_21E00 */
    ps2000_op_get_str_t         get_string_param_b;     /* sub_220B0 */
    ps2000_op_get_str_t         get_string_param_c;     /* sub_22210 */
    ps2000_op_get_str_t         get_string_param_d;     /* sub_22330 */
    ps2000_op_get_str_t         get_string_param_e;     /* sub_22380 */
    ps2000_op_get_str_t         get_string_param_f;     /* sub_225A0 */
    ps2000_op_get_str_t         get_string_param_g;     /* sub_21C70 */

    /* State and mode */
    ps2000_op_ctx_t             check_device_state;     /* sub_14610 */
    ps2000_op_ctx_t             check_channel;          /* sub_14670 */
    ps2000_op_ctx_t             get_state;              /* sub_17200 */
    ps2000_op_ctx_t             get_state_ext;          /* sub_16EE0 */
    ps2000_op_ctx_t             set_global_params;      /* sub_178A0 */
    ps2000_op_ctx_t             set_global_params_ext;  /* sub_17DC0 */

    /* Additional processing */
    ps2000_op_ctx_ptr_u32_t     process_data_ext;       /* sub_157A0 */
    ps2000_op_ctx_u32_t         config_a;               /* sub_167E0 */
    ps2000_op_ctx_ptr_u32_t     process_b;              /* sub_16AF0 */
    ps2000_op_ctx_ptr_u32_t     process_c;              /* sub_16BD0 */
    ps2000_op_ctx_u32_t         calc_offset;            /* sub_16FC0 */
    ps2000_op_ctx_t             check_readonly;         /* sub_138A0 */
    ps2000_op_ctx_t             delegate_a;             /* sub_13D80 */

    /* FIX #1: Advanced Trigger Delegates */
    ps2000_op_adv_trig_props_t  set_trigger_properties; /* sub_26800 delegate */
    ps2000_op_adv_trig_conds_t  set_trigger_conditions; /* sub_268E0 delegate */
    ps2000_op_adv_trig_dirs_t   set_trigger_directions; /* sub_269C0 delegate */
    ps2000_op_pwq_t             set_pwq;                /* sub_26AA0 delegate */
    ps2000_op_adv_trig_delay_t  set_trigger_delay;      /* sub_26B80 delegate */

    /* FIX #1: Streaming Delegate */
    ps2000_op_start_streaming_t start_streaming;        /* sub_252A0 delegate */

    /* Stub methods (originally xor eax, eax; retn) */
    ps2000_op_void_t            stub_01;                /* sub_2ADA0 */
    ps2000_op_void_t            stub_02;                /* sub_2CC10 */
    ps2000_op_void_t            stub_03;                /* nullsub_5 */
    ps2000_op_void_t            stub_04;                /* sub_2D900 */
    ps2000_op_void_t            stub_05;                /* sub_2D8F0 */
    ps2000_op_void_t            stub_06;                /* sub_2D910 */
    ps2000_op_void_t            stub_07;                /* sub_2CC30 */
    ps2000_op_void_t            stub_08;                /* sub_2CCF0 */
    ps2000_op_void_t            stub_09;                /* sub_2CEE0 */
    ps2000_op_void_t            stub_10;                /* sub_2CED0 */
    ps2000_op_void_t            stub_11;                /* sub_12B00 */
    ps2000_op_void_t            stub_12;                /* sub_136C0 */
    ps2000_op_void_t            stub_13;                /* sub_136B0 */
    ps2000_op_void_t            stub_14;                /* sub_136D0 */
    ps2000_op_void_t            stub_15;                /* sub_13D90 */
    ps2000_op_void_t            stub_16;                /* sub_13DA0 */
    ps2000_op_void_t            stub_17;                /* sub_15A00 */
    ps2000_op_void_t            stub_18;                /* sub_171C0 */
    ps2000_op_void_t            stub_19;                /* sub_171E0 */
    ps2000_op_void_t            stub_20;                /* sub_171F0 */
    ps2000_op_void_t            stub_21;                /* sub_171D0 */
    ps2000_op_void_t            stub_22;                /* sub_17390 */
    ps2000_op_void_t            stub_23;                /* sub_173A0 */
    ps2000_op_void_t            stub_24;                /* sub_173B0 */
    ps2000_op_void_t            stub_25;                /* sub_173C0 */
    ps2000_op_void_t            stub_26;                /* nullsub_1 */
    ps2000_op_void_t            stub_27;                /* nullsub_2 */
} ps2000_operations_t;

/* ========================================================================== */
/*                     EMBEDDED SUB-STRUCTURES                                */
/* ========================================================================== */
/**
 * @brief USB host wrapper (libusb context management).
 * @note Size ~0xF8 bytes. Reconstructed from sub_4EDC0/sub_4EEA0.
 */
typedef struct _ps2000_usb_host {
    libusb_context             *ctx;                /* +0x00 */
    libusb_device              *dev;                /* +0x08 */
    libusb_device_handle       *handle;             /* +0x10 */
    libusb_config_descriptor   *active_config;      /* +0x18 */
    uint8_t                     gap_20[0xD8];       /* internal state */
} ps2000_usb_host_t;

/**
 * @brief USB device abstraction (IFirmwareInterface implementation).
 * @note Size 0x148 bytes (from mov esi, 148h before operator delete).
 * FIX #100: Added semantic name for field at offset 0x40.
 */
typedef struct _ps2000_usb_device {
    void                       *vtable;             /* +0x000: IFirmwareInterface */
    libusb_context             *ctx;                /* +0x008 */
    libusb_device              *dev;                /* +0x010 */
    libusb_device_handle       *handle;             /* +0x018 */
    uint8_t                     serial[32];         /* +0x020 */
    uint32_t                    device_state_flags; /* +0x040: was gap_040[0..3] */
    uint8_t                     gap_044[0x104];     /* +0x044 */
} ps2000_usb_device_t;

/**
 * @brief Synchronization block (mutex + cond + semaphore).
 * @note Reconstructed from sub_4CB90 semaphore release pattern.
 */
typedef struct _ps2000_sync_block {
    pthread_mutex_t             mutex;              /* +0x00 */
    pthread_cond_t              cond;               /* +0x28 */
    int32_t                     semaphore_count;    /* +0x50 */
    int32_t                     semaphore_limit;    /* +0x54 */
    void                       *owner_thread;       /* +0x58 */
} ps2000_sync_block_t;

/**
 * @brief Streaming collection aggregator.
 * @note RTTI: 19StreamingCollectionIsE.
 */
typedef struct _ps2000_stream_collection {
    void                       *collection_impl;    /* +0x00 */
    volatile int32_t            ref_count;          /* +0x08 */
    uint32_t                    capacity;           /* +0x0C */
    uint8_t                     gap_10[0x18];       /* +0x10 */
} ps2000_stream_collection_t;

/**
 * @brief Capture/trigger block (core oscilloscope state).
 * @note Largest functional block (~0x140 bytes).
 */
typedef struct _ps2000_capture_block {
    double                      sample_interval;    /* +0x000 */
    uint32_t                    buffer_size;        /* +0x008 */
    uint32_t                    pre_trigger;        /* +0x00C */
    uint16_t                    timebase_index;     /* +0x010 */
    uint8_t                     channel_mask;       /* +0x012 */
    uint8_t                     trigger_enabled;    /* +0x013 */
    uint16_t                    trigger_threshold;  /* +0x014 */
    uint16_t                    trig_alt_channel;   /* +0x016 */
    uint8_t                     gap_018[0x57];      /* +0x018 */
    double                      decimation_divisor; /* +0x06F */
    uint8_t                     gap_077[0x01];      /* +0x077 */
    int16_t                    *channel_buffers[PS2000_MAX_CHANNELS]; /* +0x078 */
    void                       *capture_buffer_a;   /* +0x098 */
    void                       *capture_buffer_b;   /* +0x0A0 */
    uint8_t                     gap_0A8[0x09];      /* +0x0A8 */
    double                      residual_time;      /* +0x0B1 */
    uint8_t                     gap_0B9[0x26];      /* +0x0B9 */
    int32_t                     capture_state;      /* +0x0DF */
    uint8_t                     gap_0E3[0x04];      /* +0x0E3 */
    void                       *owner_ref;          /* +0x0E7 */
    void                       *disposable_resource;/* +0x0EF */
    uint16_t                    pending_ops;        /* +0x0F7 */
    uint8_t                     gap_0F9[0x16];      /* +0x0F9 */
    int32_t                     acquisition_mode;   /* +0x10F */
    uint8_t                     gap_113[0x02];      /* +0x113 */
    uint16_t                    trig_alt_param_a;   /* +0x115 */
    uint16_t                    trig_alt_param_b;   /* +0x117 */
    uint8_t                     gap_119[0x27];      /* +0x119 */
} ps2000_capture_block_t;

/**
 * @brief Device info and calibration block.
 * @note Serial string at offset 0x23D from context base.
 */
typedef struct _ps2000_info_block {
    char                        serial_string[32];  /* +0x00 */
    char                        variant_string[16]; /* +0x20 */
    int16_t                     cal_offset[PS2000_MAX_CHANNELS]; /* +0x30 */
    double                      cal_gain[PS2000_MAX_CHANNELS];   /* +0x38 */
    uint8_t                     eeprom_valid;       /* +0x58 */
    uint8_t                     fpga_loaded;        /* +0x59 */
    uint8_t                     capabilities;       /* +0x5A: capabilities bitmap (sub_2A850) */
    uint8_t                     gap_5B[0x25];       /* +0x5B */
} ps2000_info_block_t;

/**
 * @brief Signal generator block.
 * @note Fields at 0x392, 0x39E from sub_2ADC0.
 */
typedef struct _ps2000_siggen_block {
    uint32_t                    config;             /* +0x00 */
    double                      freq_hz;            /* +0x04 */
    double                      phase;              /* +0x0C */
    uint16_t                    awg_buffer_len;     /* +0x14 */
    uint8_t                     gap_16[0x7C];       /* +0x16 */
    uint16_t                    ext_param;          /* +0x92 */
    uint8_t                     gap_94[0x0A];       /* +0x94 */
    uint16_t                    awg_config_flag;    /* +0x9E */
} ps2000_siggen_block_t;

/**
 * @brief Streaming block (ring buffer and trigger channels).
 * @note Fields at 0x5C4-0x5E4 from sub_2ADC0/sub_2B6FD.
 */
typedef struct _ps2000_streaming_block {
    void                       *stream_buffer;      /* +0x00 */
    uint32_t                    stream_buf_size;    /* +0x08 */
    volatile uint32_t           stream_write_pos;   /* +0x0C */
    uint8_t                     gap_10[0xB4];       /* +0x10 */
    uint8_t                     channel_a_flag;     /* +0xC4 */
    uint8_t                     gap_C5[0x02];       /* +0xC5 */
    uint8_t                     trigger_channel_sel;/* +0xC7 */
    uint16_t                    channel_b_config;   /* +0xC8 */
    uint8_t                     gap_CA[0x0A];       /* +0xCA */
    uint8_t                     trig_channel_a;     /* +0xD4 */
    uint8_t                     trig_channel_b;     /* +0xD5 */
    uint8_t                     gap_D6[0x0C];       /* +0xD6 */
    uint16_t                    adc_spi_cache[PS2000_MAX_CHANNELS]; /* +0xE2 */
} ps2000_streaming_block_t;

/* ========================================================================== */
/*                         TOP-LEVEL CONTEXT                                  */
/* ========================================================================== */
struct _ps2000_context {
    /* 0x000 */ ps2000_usb_host_t          usb_host;
    /* 0x0F8 */ ps2000_usb_device_t        usb_device;
    /* 0x240 */ ps2000_sync_block_t        sync;
    /* 0x280 */ ps2000_stream_collection_t stream_collection;
    /* 0x2C0 */ ps2000_capture_block_t     capture;
    /* 0x400 */ ps2000_info_block_t        info;
    /* 0x480 */ ps2000_siggen_block_t      siggen;
    /* 0x520 */ ps2000_streaming_block_t   streaming;

    /* Device variant and operations table (from sub_237C0) */
    /* 0x60A */ int32_t                    device_variant;    /* +0xAA relative */
    /* 0x60E */ uint8_t                    valid_flag;        /* +0xBC relative */
    /* 0x60F */ uint8_t                    gap_60F[0x03];
    /* 0x612 */ int32_t                    device_mode;       /* +0xF8 relative */
    /* 0x616 */ uint8_t                    gap_616[0x02];
    /* 0x618 */ ps2000_operations_t        ops;               /* Operations table */

    /* Reference counting */
    volatile uint32_t              ref_count;
    uint8_t                        active_flag;
    uint8_t                        gap_end[0x03];
};

/* ========================================================================== */
/*                         GLOBAL STATE                                       */
/* ========================================================================== */
typedef struct _ps2000_global_state {
    ps2000_context_t            *contexts[PS2000_MAX_UNITS];
    uint32_t                     num_active_units;
    pthread_mutex_t              global_mutex;
    bool                         initialized;
} ps2000_global_state_t;

/* ========================================================================== */
/*                           LOGGING                                          */
/* ========================================================================== */
typedef enum {
    PS2000_LOG_DEBUG = 0,
    PS2000_LOG_INFO  = 1,
    PS2000_LOG_WARN  = 2,
    PS2000_LOG_ERROR = 3
} ps2000_log_level_t;

void _ps2000_log_init(void);
void _ps2000_log_write(ps2000_log_level_t level, const char *fmt, ...);
void ps2000_set_log_callback(void (*cb)(const char *msg));
void ps2000_set_log_level(ps2000_log_level_t level);

#define PS2000_LOG_DBG(...) _ps2000_log_write(PS2000_LOG_DEBUG, __VA_ARGS__)
#define PS2000_LOG_INF(...) _ps2000_log_write(PS2000_LOG_INFO,  __VA_ARGS__)
#define PS2000_LOG_WRN(...) _ps2000_log_write(PS2000_LOG_WARN,  __VA_ARGS__)
#define PS2000_LOG_ERR(...) _ps2000_log_write(PS2000_LOG_ERROR, __VA_ARGS__)

/* ========================================================================== */
/*                    INTERNAL FUNCTION PROTOTYPES                            */
/* ========================================================================== */

/* --- Core / Init (_ps2000_init_*) --- */
ps2000_status_t _ps2000_init_global_state(void);
void            _ps2000_init_cleanup_global(void);
ps2000_status_t _ps2000_init_context(ps2000_context_t **ctx);
void            _ps2000_init_destroy_context(ps2000_context_t *ctx);

/* --- Core registration (FIX #6) --- */
int             _ps2000_core_register_context(ps2000_context_t *ctx);
void            _ps2000_core_unregister_context(ps2000_context_t *ctx);

/* --- Operations dispatcher (_ps2000_ops_*) --- */
ps2000_status_t _ps2000_ops_init_table(ps2000_context_t *ctx);
int             _ps2000_ops_stub_zero(void);
int             _ps2000_ops_get_identity(ps2000_context_t *ctx);
int             _ps2000_ops_get_max_buffer_size(ps2000_context_t *ctx);
int             _ps2000_ops_get_mode_count(ps2000_context_t *ctx);
bool            _ps2000_ops_check_capability(ps2000_context_t *ctx, uint8_t bit);
int             _ps2000_ops_validate_channels(ps2000_context_t *ctx,
                                              uint32_t ch_a, uint32_t ch_b);
int             _ps2000_ops_check_awg(ps2000_context_t *ctx);
int             _ps2000_ops_siggen_reset(ps2000_context_t *ctx);
int             _ps2000_ops_check_timebase_mode(ps2000_context_t *ctx);
int             _ps2000_ops_get_state(ps2000_context_t *ctx);
int             _ps2000_ops_check_device_state(ps2000_context_t *ctx);
void            _ps2000_ops_update_device_state(ps2000_context_t *ctx);
int             _ps2000_ops_siggen_default_init(ps2000_context_t *ctx);
int             _ps2000_ops_check_channel(ps2000_context_t *ctx);
/* FIX #16: Extended validation with frequency check */
int             _ps2000_ops_validate_siggen_freq(ps2000_context_t *ctx,
                                                 double freq_normalized,
                                                 uint32_t awg_samples);

/* --- USB Transport (_ps2000_usb_*) --- */
ps2000_status_t _ps2000_usb_open_device(ps2000_context_t *ctx, uint32_t serial);
void            _ps2000_usb_close_device(ps2000_context_t *ctx);
ps2000_status_t _ps2000_usb_control_transfer(ps2000_context_t *ctx,
                                             uint8_t req_type, uint8_t req,
                                             uint16_t val, uint16_t idx,
                                             uint8_t *data, uint16_t len);
ps2000_status_t _ps2000_usb_bulk_transfer(ps2000_context_t *ctx, uint8_t ep,
                                          const uint8_t *data, uint32_t len);

/* --- HW / IO commands (_ps2000_hw_*) --- */
ps2000_status_t _ps2000_hw_init_sequence(ps2000_context_t *ctx);
/* FIX #13: reg parameter changed to uint16_t */
ps2000_status_t _ps2000_hw_write_reg(ps2000_context_t *ctx, uint16_t reg, uint16_t val);
ps2000_status_t _ps2000_hw_read_reg(ps2000_context_t *ctx, uint16_t reg, uint16_t *val);
ps2000_status_t _ps2000_hw_load_fpga(ps2000_context_t *ctx,
                                     const uint8_t *bitstream, uint32_t len);
ps2000_status_t _ps2000_hw_adc_spi_write(ps2000_context_t *ctx,
                                         uint8_t ch, uint16_t val);
ps2000_status_t _ps2000_hw_io_cmd(ps2000_context_t *ctx, ps2000_io_cmd_t cmd,
                                  uint16_t addr, uint16_t value);
void            _ps2000_hw_cleanup(ps2000_context_t *ctx);
/* FIX #15: PreLaunch */
ps2000_status_t _ps2000_hw_prelaunch(void);
/* FIX #15: Apply fix */
int             _ps2000_hw_apply_fix(uint32_t magic, uint16_t version);

/* --- Init table parser (_ps2000_init_tbl_*) --- */
ps2000_status_t _ps2000_init_tbl_execute(ps2000_context_t *ctx,
                                         const uint8_t *script, uint32_t script_len);

/* --- Firmware (_ps2000_fw_*) --- */
ps2000_status_t _ps2000_fw_init(ps2000_context_t *ctx);
void            _ps2000_fw_cleanup(ps2000_context_t *ctx);
ps2000_status_t _ps2000_fw_load(ps2000_context_t *ctx,
                                const uint8_t *fw_data, uint32_t fw_len);
ps2000_status_t _ps2000_fw_reset_device(ps2000_context_t *ctx);
ps2000_status_t _ps2000_fw_verify(ps2000_context_t *ctx);

/* --- Threading (_ps2000_thr_*) --- */
ps2000_status_t _ps2000_thr_create(pthread_t *thr, const char *name,
                                   void (*func)(void *), void *arg);
ps2000_status_t _ps2000_thr_join(pthread_t *thr);
bool            _ps2000_thr_is_running(const pthread_t *thr);

/* --- Math (_ps2000_math_*) --- */
double   _ps2000_math_round_fast(double val);
double   _ps2000_math_adc_normalize(int16_t raw);
int16_t  _ps2000_math_adc_denormalize(double normalized);
double   _ps2000_math_timebase_interval(uint32_t index);
int32_t  _ps2000_math_timing_calc(int32_t raw_val, double interval, double offset);
int16_t  _ps2000_math_adc_convert(int16_t raw, double gain, double corr_offset);
uint32_t _ps2000_math_stream_pos(uint32_t base, uint32_t increment, double scale);
double   _ps2000_math_awg_to_dac(double normalized);
double   _ps2000_math_dac_to_awg(uint16_t dac_code);
double   _ps2000_math_sine_sample(uint32_t index, uint32_t total_samples);

/* --- Config & Calibration (_ps2000_cfg_*) --- */
ps2000_status_t _ps2000_cfg_load_eeprom(ps2000_context_t *ctx);
ps2000_status_t _ps2000_cfg_apply_calibration(ps2000_context_t *ctx,
                                              int16_t *raw_data,
                                              uint32_t num_samples);
/* FIX #6: Added missing prototype */
ps2000_status_t _ps2000_cfg_get_device_info(ps2000_context_t *ctx,
                                            char *serial_buf, uint32_t buf_len,
                                            uint32_t *hw_version);

/* --- Trigger (_ps2000_trg_*) --- */
ps2000_status_t _ps2000_trg_validate_config(ps2000_context_t *ctx,
                                            uint16_t channel_mask,
                                            uint16_t threshold,
                                            uint16_t direction,
                                            uint16_t delay);
ps2000_status_t _ps2000_trg_calculate_position(ps2000_context_t *ctx,
                                               uint32_t *trig_pos_out);

/* --- Streaming (_ps2000_strm_*) --- */
ps2000_status_t _ps2000_strm_process_raw(ps2000_context_t *ctx,
                                         const int16_t *raw_buf,
                                         uint32_t num_samples,
                                         int16_t *out_buf,
                                         uint32_t *processed_count);
uint32_t        _ps2000_strm_update_ring_pos(uint32_t current_pos,
                                             uint32_t increment,
                                             uint32_t buf_size);

/* --- Oscilloscope (_ps2000_osc_*) --- */
ps2000_status_t _ps2000_osc_calc_timebase(uint32_t index, double *interval_ns);
ps2000_status_t _ps2000_osc_validate_buffer(uint32_t pre_trig,
                                            uint32_t post_trig,
                                            uint32_t max_samples);
/* FIX #6: Added missing prototypes */
ps2000_status_t _ps2000_osc_get_timebase(ps2000_context_t *ctx,
                                         uint32_t timebase_index, uint32_t num_samples,
                                         double *interval_ns, uint32_t *max_samples);
ps2000_status_t _ps2000_osc_run_block(ps2000_context_t *ctx,
                                      uint32_t pre_trigger_samples, uint32_t post_trigger_samples);
ps2000_status_t _ps2000_osc_get_values(ps2000_context_t *ctx,
                                       uint32_t channel, int16_t *buffer, uint32_t start_index,
                                       uint32_t num_samples, uint32_t *returned_samples);
void _ps2000_osc_compute_decim_indices(int16_t ref_sample,
                                       int16_t secondary_sample, int32_t carry,
                                       int32_t *out_primary, int32_t *out_secondary, int32_t *out_div256);

/* --- Signal Generator (_ps2000_gen_*) --- */
/* FIX #6: Added missing prototypes */
ps2000_status_t _ps2000_gen_set_builtin_internal(ps2000_context_t *ctx,
                                                 double freq_hz, uint32_t wave_type, double amplitude, double offset);
ps2000_status_t _ps2000_gen_set_arbitrary_internal(ps2000_context_t *ctx,
                                                   const double *waveform, uint32_t num_samples, double normalized_freq);
ps2000_status_t ps2000_gen_create_waveform(double *buffer,
                                           uint32_t num_samples, uint32_t wave_type);
ps2000_status_t ps2000_set_sig_gen_off(ps2000_context_t *ctx);

/* --- HW Extended API (FIX #6) --- */
ps2000_status_t _ps2000_hw_reset(ps2000_context_t *ctx);
ps2000_status_t _ps2000_hw_flash_read_page(ps2000_context_t *ctx,
                                           uint16_t page, uint8_t *data, uint16_t data_len);
ps2000_status_t _ps2000_hw_flash_erase_page(ps2000_context_t *ctx, uint16_t page);
ps2000_status_t _ps2000_hw_flash_protect(ps2000_context_t *ctx);
ps2000_status_t _ps2000_hw_eeprom_read(ps2000_context_t *ctx,
                                       uint16_t offset, uint8_t *data, uint16_t data_len);
ps2000_status_t _ps2000_hw_eeprom_wait(ps2000_context_t *ctx);

/* --- Utilities (_ps2000_util_*) --- */
ps2000_status_t _ps2000_util_mutex_lock_safe(pthread_mutex_t *mtx,
                                             uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* PS2000_INTERNAL_H */