#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

// Global debug flag - set by command line arguments
extern int debug_enabled;

// Debug macro that only prints when debug is enabled
#define DEBUG_PRINT(fmt, ...) \
    do { \
        if (debug_enabled) { \
            fprintf(stderr, "DEBUG: " fmt, ##__VA_ARGS__); \
        } \
    } while (0)

// Initialize debug system
void debug_init(void);

#endif // DEBUG_H
