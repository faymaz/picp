#include "k150.h"

// Eğer bu fonksiyonlar eksikse
int DoDetectChip_Enhanced(char** detected_name) {
    *detected_name = strdup("PIC16F628A");
    return 0;
}

int DoErasePgm_Enhanced(const char* device_name) {
    return 0;
}

void k150_list_devices(void) {
    printf("PIC16F84A\nPIC16F628A\nPIC16F876A\nPIC16F887\n");
}

// Protocol functions stubs
int k150_detect_protocol(const char *port) { return 3; }
int k150_program_auto(const char *port, const char *device, 
                      const unsigned char *rom, int size, 
                      unsigned int config) { return 0; }
int k150_erase_auto(const char *port, const char *device) { return 0; }
int k150_read_auto(const char *port, const char *device, 
                   unsigned char *buffer, int size) { return 0; }
void k150_test_protocols(const char *port) { printf("Test stub\n"); }
