// k150_integration.h - Integration header for enhanced K150 support

#ifndef __K150_INTEGRATION_H__
#define __K150_INTEGRATION_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// Protocol types
typedef enum {
    K150_PROTO_AUTO = 0,    // Auto-detect
    K150_PROTO_P018,        // Standard P018
    K150_PROTO_P18A,        // Enhanced P18A
    K150_PROTO_MICROPRO     // Micropro.exe compatible
} k150_protocol_t;

// Operation results
typedef enum {
    K150_OK = 0,
    K150_ERR_PORT,
    K150_ERR_PROTOCOL,
    K150_ERR_DEVICE,
    K150_ERR_TIMEOUT,
    K150_ERR_VERIFY,
    K150_ERR_MEMORY
} k150_result_t;

// Device capabilities
typedef struct {
    const char *name;
    uint16_t device_id;
    uint16_t rom_size;        // in words
    uint16_t eeprom_size;     // in bytes
    uint16_t config_addr;
    uint16_t config_mask;
    bool has_calibration;
    uint16_t cal_addr;
} k150_device_info_t;

// Programming options
typedef struct {
    k150_protocol_t protocol;
    bool verify_after_write;
    bool preserve_eeprom;
    bool preserve_calibration;
    bool use_icsp;
    int retry_count;
    int timeout_ms;
} k150_options_t;

// Statistics
typedef struct {
    uint32_t operations;
    uint32_t bytes_written;
    uint32_t bytes_read;
    uint32_t verifications;
    double time_elapsed;
} k150_stats_t;

// Global state
extern struct {
    bool is_initialized;
    k150_protocol_t active_protocol;
    k150_options_t options;
    k150_device_info_t device_info;
    k150_stats_t stats;
    clock_t start_time;
} g_k150_state;

// External function declarations from existing code
extern int k150_detect_protocol(const char *port);
extern int DoDetectChip_Enhanced(char **detected_name);
extern int k150_program_auto(const char *port, const char *device, 
                             const unsigned char *rom, int size, unsigned int config);
extern int k150_erase_auto(const char *port, const char *device);
extern int k150_read_auto(const char *port, const char *device, unsigned char *buffer, int size);

// Main API function declarations
k150_result_t k150_init(const char *port, k150_options_t *options);
k150_result_t k150_detect_device(k150_device_info_t *info);
k150_result_t k150_erase_device(void);
k150_result_t k150_read_program(uint8_t *buffer, size_t size);
k150_result_t k150_write_program(const uint8_t *buffer, size_t size);
k150_result_t k150_get_stats(k150_stats_t *stats);
void k150_cleanup(void);
const char* k150_error_string(k150_result_t result);
const char* k150_protocol_name(k150_protocol_t protocol);
void k150_print_device_info(const k150_device_info_t *info);

#endif /* __K150_INTEGRATION_H__ */