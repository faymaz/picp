#include "debug.h"

// Global debug flag - disabled by default
int debug_enabled = 0;

// Initialize debug system
void debug_init(void)
{
    debug_enabled = 0;
}
