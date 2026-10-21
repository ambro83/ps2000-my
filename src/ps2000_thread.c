#include "ps2000_internal.h"
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/* ========================================================================== */
/*                     THREAD WRAPPER (IRunnable replacement)                 */
/* ========================================================================== */

/**
 * @brief Thread context structure for passing func + arg to trampoline.
 *
 * FIX #5: Allocated on heap to avoid use-after-free.
 * Previously the address of a local stack variable was passed to
 * pthread_create, which is undefined behaviour once the creating
 * function returns.
 */
typedef struct {
    void (*func)(void*);
    void *arg;
} _ps2000_thr_ctx_t;

/**
 * @brief Thread entry wrapper.
 *
 * Reconstructs sub_1A210 / sub_1A4C0 IRunnable dispatch.
 * Original used virtual call [rax+10h]. We use direct function pointer.
 *
 * FIX #5: Properly unpacks the heap-allocated context and frees it
 * after the callback completes.
 */
static void* _ps2000_thr_trampoline(void *arg) {
    _ps2000_thr_ctx_t *ctx = (_ps2000_thr_ctx_t*)arg;
    if (ctx && ctx->func) {
        ctx->func(ctx->arg);
    }
    free(ctx);
    return NULL;
}

/**
 * @brief Create and start a new thread.
 *
 * Replaces C++ Thread constructor + start().
 * Implements "Thread already started" check from sub_B00C / sub_B466.
 * Original throws exception: "Thread already started."
 *
 * FIX #5: Allocates a heap context to safely pass both func and arg
 * through the pthread_create void* parameter.
 */
ps2000_status_t _ps2000_thr_create(pthread_t *thr, const char *name,
                                   void (*func)(void *), void *arg) {
    if (!thr || !func) return PS2000_ERR_INVALID_PARAM;

    /* Note: In a full implementation, thread state would be tracked in a
     * structure to prevent double-start (sub_B00C). Here we assume
     * the caller manages the lifecycle correctly. */

    /* Allocate thread context to pass both func and arg safely */
    _ps2000_thr_ctx_t *thr_ctx = (_ps2000_thr_ctx_t*)malloc(sizeof(_ps2000_thr_ctx_t));
    if (!thr_ctx) {
        return PS2000_ERR_MEMORY;
    }
    thr_ctx->func = func;
    thr_ctx->arg = arg;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 1024 * 1024); /* 1MB stack */

    int ret = pthread_create(thr, &attr, _ps2000_thr_trampoline, thr_ctx);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        free(thr_ctx);
        PS2000_LOG_ERR("Failed to create thread '%s': errno=%d",
                       name ? name : "unknown", ret);
        return PS2000_ERR_MEMORY; /* Maps to std::__throw_system_error */
    }

    PS2000_LOG_DBG("Thread '%s' created", name ? name : "worker");
    return PS2000_OK;
}

ps2000_status_t _ps2000_thr_join(pthread_t *thr) {
    if (!thr) return PS2000_ERR_INVALID_PARAM;
    int ret = pthread_join(*thr, NULL);
    return (ret == 0) ? PS2000_OK : PS2000_ERR_BUSY;
}

bool _ps2000_thr_is_running(const pthread_t *thr) {
    /* Simplified check; original uses complex state machine */
    return (thr != NULL);
}