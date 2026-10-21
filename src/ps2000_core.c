#include "ps2000_internal.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>

/* ========================================================================== */
/*                          GLOBAL STATE                                      */
/* ========================================================================== */

static ps2000_global_state_t g_ps2000_state = {0};
static pthread_once_t        g_init_once = PTHREAD_ONCE_INIT;

/**
 * @brief Internal one-time global state initialization.
 *
 * Reconstructed from global constructor pattern and pthread_once usage.
 * Initializes the global mutex, logging subsystem, and sets the
 * initialized flag.
 */
static void _ps2000_init_global_impl(void) {
    memset(&g_ps2000_state, 0, sizeof(g_ps2000_state));

    if (pthread_mutex_init(&g_ps2000_state.global_mutex, NULL) != 0) {
        g_ps2000_state.initialized = false;
        return;
    }

    /* Initialize logging subsystem before any other operations */
    _ps2000_log_init();

    g_ps2000_state.initialized = true;
    PS2000_LOG_INF("PS2000 library initialized, driver version %s",
                   PS2000_DRIVER_VERSION);
}

/**
 * @brief Ensure global state is initialized (thread-safe).
 */
ps2000_status_t _ps2000_init_global_state(void) {
    if (pthread_once(&g_init_once, _ps2000_init_global_impl) != 0) {
        return PS2000_ERR_MEMORY;
    }
    return g_ps2000_state.initialized ? PS2000_OK : PS2000_ERR_MEMORY;
}

/**
 * @brief Clean up global state and all active contexts.
 *
 * Iterates through all PS2000_MAX_UNITS (127) context slots and
 * destroys any active contexts. Called during library shutdown.
 */
void _ps2000_init_cleanup_global(void) {
    if (!g_ps2000_state.initialized) return;

    pthread_mutex_lock(&g_ps2000_state.global_mutex);

    for (uint32_t i = 0; i < PS2000_MAX_UNITS; i++) {
        if (g_ps2000_state.contexts[i] != NULL) {
            _ps2000_init_destroy_context(g_ps2000_state.contexts[i]);
            g_ps2000_state.contexts[i] = NULL;
        }
    }

    g_ps2000_state.num_active_units = 0;
    pthread_mutex_unlock(&g_ps2000_state.global_mutex);
    pthread_mutex_destroy(&g_ps2000_state.global_mutex);
    g_ps2000_state.initialized = false;

    PS2000_LOG_INF("PS2000 library shutdown complete");
}

/* ========================================================================== */
/*                       CONTEXT LIFECYCLE                                    */
/* ========================================================================== */

/**
 * @brief Atomic decrement of reference count.
 *
 * Reconstructed from the lock xadd pattern in sub_1A430:
 *   mov eax, 0FFFFFFFFh
 *   lock xadd [rbp+0Ch], eax
 *
 * @return true if the object should be freed (refcount reached 0).
 */
static bool _ps2000_core_release_ref(ps2000_context_t *ctx) {
    if (!ctx) return false;
    uint32_t old_val = __sync_fetch_and_sub(&ctx->ref_count, 1U);
    return (old_val == 1U);
}

/**
 * @brief Allocate and initialize a new device context.
 *
 * Replaces operator new(0x602) from the original code.
 * Initializes all sub-structures (USB, sync, capture, info, siggen,
 * streaming) and the operations table via _ps2000_ops_init_table.
 *
 * Default device variant is set to PS2000_DEVICE_VARIANT_2.
 * The actual variant is determined during device enumeration.
 */
ps2000_status_t _ps2000_init_context(ps2000_context_t **out_ctx) {
    ps2000_status_t status;
    ps2000_context_t *ctx = NULL;

    if (!out_ctx) return PS2000_ERR_INVALID_PARAM;

    /* Verify global initialization */
    status = _ps2000_init_global_state();
    if (status != PS2000_OK) return status;

    /* Allocate context (replaces operator new) */
    ctx = (ps2000_context_t*)calloc(1, sizeof(ps2000_context_t));
    if (!ctx) {
        PS2000_LOG_ERR("Failed to allocate context: out of memory");
        return PS2000_ERR_MEMORY;
    }

    /* Initialize synchronization primitives */
    if (pthread_mutex_init(&ctx->sync.mutex, NULL) != 0) {
        free(ctx);
        return PS2000_ERR_MEMORY;
    }
    if (pthread_cond_init(&ctx->sync.cond, NULL) != 0) {
        pthread_mutex_destroy(&ctx->sync.mutex);
        free(ctx);
        return PS2000_ERR_MEMORY;
    }

    /* Initialize semaphore (from sub_4CB90 pattern) */
    ctx->sync.semaphore_count = 0;
    ctx->sync.semaphore_limit = 1;
    ctx->sync.owner_thread = NULL;

    /* Set initial values for capture block */
    ctx->capture.sample_interval = 0.0;
    ctx->capture.buffer_size = 0;
    ctx->capture.pre_trigger = 0;
    ctx->capture.timebase_index = 0;
    ctx->capture.channel_mask = 0;
    ctx->capture.trigger_enabled = 0;
    ctx->capture.trigger_threshold = 0;
    ctx->capture.trig_alt_channel = 0;
    ctx->capture.decimation_divisor = 1.0;
    ctx->capture.residual_time = 0.0;
    ctx->capture.capture_state = 0;
    ctx->capture.acquisition_mode = 0;
    ctx->capture.trig_alt_param_a = 0;
    ctx->capture.trig_alt_param_b = 0;
    ctx->capture.owner_ref = NULL;
    ctx->capture.disposable_resource = NULL;
    ctx->capture.pending_ops = 0;

    /* Initialize channel buffer pointers */
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        ctx->capture.channel_buffers[ch] = NULL;
    }
    ctx->capture.capture_buffer_a = NULL;
    ctx->capture.capture_buffer_b = NULL;

    /* Set initial values for info block */
    ctx->info.eeprom_valid = 0;
    ctx->info.fpga_loaded = 0;
    ctx->info.capabilities = 0;
    memset(ctx->info.serial_string, 0, sizeof(ctx->info.serial_string));
    memset(ctx->info.variant_string, 0, sizeof(ctx->info.variant_string));
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        ctx->info.cal_offset[ch] = 0;
        ctx->info.cal_gain[ch] = 1.0;
    }

    /* Set initial values for siggen block */
    ctx->siggen.config = PS2000_SIGGEN_DEFAULT;  /* 0x80000 from sub_2A850 */
    ctx->siggen.freq_hz = 0.0;
    ctx->siggen.phase = 0.0;
    ctx->siggen.awg_buffer_len = 0;
    ctx->siggen.ext_param = 0;
    ctx->siggen.awg_config_flag = 0;

    /* Set initial values for streaming block */
    ctx->streaming.stream_buffer = NULL;
    ctx->streaming.stream_buf_size = 0;
    ctx->streaming.stream_write_pos = 0;
    ctx->streaming.channel_a_flag = 0;
    ctx->streaming.trigger_channel_sel = 0;
    ctx->streaming.channel_b_config = 0;
    ctx->streaming.trig_channel_a = 0;
    ctx->streaming.trig_channel_b = 0;
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        ctx->streaming.adc_spi_cache[ch] = 0;
    }

    /* Set initial device state */
    ctx->device_variant = PS2000_DEVICE_VARIANT_2;
    ctx->valid_flag = 0;
    ctx->device_mode = 0;
    ctx->active_flag = 0;
    ctx->ref_count = 1U;

    /* Initialize the operations table based on device variant */
    status = _ps2000_ops_init_table(ctx);
    if (status != PS2000_OK) {
        PS2000_LOG_WRN("Operations table initialization failed: %d", status);
        /* Continue with default operations, don't fail context creation */
    }

    *out_ctx = ctx;
    PS2000_LOG_DBG("Context created: %p", (void*)ctx);
    return PS2000_OK;
}

/**
 * @brief Destroy a device context and free all resources.
 *
 * Replaces operator delete from the original code.
 * Performs cleanup in reverse order of initialization:
 * USB -> HW -> FW -> sync primitives -> buffers -> memory.
 */
void _ps2000_init_destroy_context(ps2000_context_t *ctx) {
    if (!ctx) return;

    /* Check if we should actually free the memory */
    if (!_ps2000_core_release_ref(ctx)) {
        return; /* Other references still exist */
    }

    PS2000_LOG_DBG("Destroying context: %p", (void*)ctx);

    /* Close USB device if open */
    _ps2000_usb_close_device(ctx);

    /* Clean up hardware state */
    _ps2000_hw_cleanup(ctx);

    /* Clean up firmware subsystem */
    _ps2000_fw_cleanup(ctx);

    /* Destroy synchronization primitives */
    pthread_cond_destroy(&ctx->sync.cond);
    pthread_mutex_destroy(&ctx->sync.mutex);

    /* Free streaming buffer if allocated */
    if (ctx->streaming.stream_buffer) {
        free(ctx->streaming.stream_buffer);
        ctx->streaming.stream_buffer = NULL;
    }

    /* Free channel buffers if allocated */
    for (uint32_t ch = 0; ch < PS2000_MAX_CHANNELS; ch++) {
        if (ctx->capture.channel_buffers[ch]) {
            free(ctx->capture.channel_buffers[ch]);
            ctx->capture.channel_buffers[ch] = NULL;
        }
    }

    /* Free capture buffers */
    if (ctx->capture.capture_buffer_a) {
        free(ctx->capture.capture_buffer_a);
        ctx->capture.capture_buffer_a = NULL;
    }
    if (ctx->capture.capture_buffer_b) {
        free(ctx->capture.capture_buffer_b);
        ctx->capture.capture_buffer_b = NULL;
    }

    /* Free disposable resource */
    if (ctx->capture.disposable_resource) {
        free(ctx->capture.disposable_resource);
        ctx->capture.disposable_resource = NULL;
    }

    /* Final cleanup and deallocation */
    memset(ctx, 0, sizeof(ps2000_context_t));
    free(ctx);
}

/* ========================================================================== */
/*                     UTILITY FUNCTIONS                                      */
/* ========================================================================== */

/**
 * @brief Safe mutex lock with timeout.
 *
 * Reconstructed from sub_4C970 (pthread_mutex_timedlock wrapper).
 * Uses CLOCK_REALTIME for absolute timeout calculation.
 *
 * @param mtx       Mutex to lock
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return PS2000_OK on success, PS2000_ERR_TIMEOUT on timeout,
 *         PS2000_ERR_BUSY on other failures
 */
ps2000_status_t _ps2000_util_mutex_lock_safe(pthread_mutex_t *mtx,
                                             uint32_t timeout_ms) {
    if (!mtx) return PS2000_ERR_INVALID_PARAM;

    /* No timeout: simple blocking lock */
    if (timeout_ms == 0U) {
        return (pthread_mutex_lock(mtx) == 0) ? PS2000_OK : PS2000_ERR_BUSY;
    }

    /* Calculate absolute timeout */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += (time_t)(timeout_ms / 1000U);
    ts.tv_nsec += (long)((timeout_ms % 1000U) * 1000000L);
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    int ret = pthread_mutex_timedlock(mtx, &ts);
    if (ret == 0) return PS2000_OK;
    if (ret == ETIMEDOUT) return PS2000_ERR_TIMEOUT;
    return PS2000_ERR_BUSY;
}

/**
 * @brief Register a context in the global context array.
 *
 * Finds the first available slot in g_ps2000_state.contexts
 * and registers the context. Returns the index or -1 if full.
 *
 * @return Index in the context array, or -1 if no slots available.
 */
int _ps2000_core_register_context(ps2000_context_t *ctx) {
    if (!ctx) return -1;

    pthread_mutex_lock(&g_ps2000_state.global_mutex);

    int slot = -1;
    for (uint32_t i = 0; i < PS2000_MAX_UNITS; i++) {
        if (g_ps2000_state.contexts[i] == NULL) {
            g_ps2000_state.contexts[i] = ctx;
            slot = (int)i;
            break;
        }
    }

    if (slot >= 0) {
        g_ps2000_state.num_active_units++;
        PS2000_LOG_DBG("Context registered at slot %d (%u active)",
                       slot, g_ps2000_state.num_active_units);
    } else {
        PS2000_LOG_ERR("No available context slots (max %u)",
                       PS2000_MAX_UNITS);
    }

    pthread_mutex_unlock(&g_ps2000_state.global_mutex);
    return slot;
}

/**
 * @brief Unregister a context from the global context array.
 */
void _ps2000_core_unregister_context(ps2000_context_t *ctx) {
    if (!ctx) return;

    pthread_mutex_lock(&g_ps2000_state.global_mutex);

    for (uint32_t i = 0; i < PS2000_MAX_UNITS; i++) {
        if (g_ps2000_state.contexts[i] == ctx) {
            g_ps2000_state.contexts[i] = NULL;
            if (g_ps2000_state.num_active_units > 0) {
                g_ps2000_state.num_active_units--;
            }
            PS2000_LOG_DBG("Context unregistered from slot %u (%u active)",
                           i, g_ps2000_state.num_active_units);
            break;
        }
    }

    pthread_mutex_unlock(&g_ps2000_state.global_mutex);
}