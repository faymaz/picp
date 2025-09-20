// k150_p18a_stub.c - Temporary stub functions to fix linking

#include "k150_integration.h"
#include <stdio.h>

// Stub implementations to satisfy linking
int k150_p18a_init(const char *port) {
    printf("k150_p18a_init: STUB - not implemented\n");
    return ERROR;
}

void k150_p18a_close(void) {
    printf("k150_p18a_close: STUB - not implemented\n");
}

int k150_p18a_program_device(const char *device, const unsigned char *data, int size, unsigned int config) {
    printf("k150_p18a_program_device: STUB - not implemented\n");
    return ERROR;
}

int k150_p18a_erase_chip(const char *device) {
    printf("k150_p18a_erase_chip: STUB - not implemented\n");
    return ERROR;
}

int k150_p18a_read_device(const char *device, unsigned char *buffer, int size) {
    printf("k150_p18a_read_device: STUB - not implemented\n");
    return ERROR;
}
