#include "ps2000_internal.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

/* ========================================================================== */
/*                     LOGGING SUBSYSTEM                                      */
/* ========================================================================== */

static ps2000_log_level_t g_log_level = PS2000_LOG_INFO;
static void (*g_log_callback)(const char *msg) = NULL;

void _ps2000_log_init(void) {
    g_log_level = PS2000_LOG_INFO;
    g_log_callback = NULL;
}

void ps2000_set_log_callback(void (*cb)(const char *msg)) {
    g_log_callback = cb;
}

void ps2000_set_log_level(ps2000_log_level_t level) {
    g_log_level = level;
}

void _ps2000_log_write(ps2000_log_level_t level, const char *fmt, ...) {
    if (level < g_log_level) return;

    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (g_log_callback) {
        g_log_callback(buffer);
    } else {
        const char *lvl_str = "INFO";
        if (level == PS2000_LOG_DEBUG) lvl_str = "DEBUG";
        else if (level == PS2000_LOG_WARN) lvl_str = "WARN";
        else if (level == PS2000_LOG_ERROR) lvl_str = "ERROR";

        fprintf(stderr, "[PS2000 %s] %s\n", lvl_str, buffer);
    }
}