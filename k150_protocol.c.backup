// k150_protocol.c - Protocol auto-detection and selection
// Combines P018, P18A and Micropro.exe protocols

#include "k150.h"
#include "k150_integration.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

// Constants for compatibility
#ifndef SUCCESS
#define SUCCESS 0
#endif
#ifndef ERROR
#define ERROR 1
#endif

// Debug macro
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf("[PROTOCOL] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

// Protocol types
typedef enum {
    PROTOCOL_UNKNOWN = 0,
    PROTOCOL_P014,      // Old protocol
    PROTOCOL_P016,      // Legacy protocol  
    PROTOCOL_P018,      // Current standard protocol
    PROTOCOL_P18A,      // Enhanced protocol
    PROTOCOL_MICROPRO   // Micropro.exe compatible
} K150_PROTOCOL;

// Protocol detection result
typedef struct {
    K150_PROTOCOL protocol;
    int firmware_version;
    char version_string[32];
    int supports_icsp;
    int max_rom_size;
    int max_eeprom_size;
} K150_INFO;

// External protocol functions
extern int k150_p18a_init(const char *port);
extern void k150_p18a_close(void);
extern int k150_p18a_program_device(const char *device, const unsigned char *rom, int size, unsigned int config);

extern int micropro_open(const char *port);
extern void micropro_close(void);
extern int micropro_program_device(const char *device, const unsigned char *rom, int size, unsigned int config);

// Global protocol info
static K150_INFO k150_info = {PROTOCOL_UNKNOWN, 0, "", 0, 0, 0};

// Detect K150 protocol version
int k150_detect_protocol(const char *port)
{
    printf("K150: Auto-detecting protocol version...\n");
    
    // Try P18A first (newest)
    printf("K150: Trying P18A protocol...\n");
    if (k150_p18a_init(port) == SUCCESS) {
        k150_info.protocol = PROTOCOL_P18A;
        k150_info.firmware_version = 0x30;
        strcpy(k150_info.version_string, "P18A Enhanced");
        k150_info.supports_icsp = 1;
        k150_info.max_rom_size = 32768;
        k150_info.max_eeprom_size = 256;
        printf("K150: ✓ P18A protocol detected\n");
        k150_p18a_close();
        return PROTOCOL_P18A;
    }
    
    // Try Micropro.exe compatibility
    printf("K150: Trying Micropro.exe protocol...\n");
    if (micropro_open(port) == SUCCESS) {
        // Try Micropro initialization
        extern int micropro_init_sequence(void);
        if (micropro_init_sequence() == SUCCESS) {
            k150_info.protocol = PROTOCOL_MICROPRO;
            k150_info.firmware_version = 0x20;
            strcpy(k150_info.version_string, "Micropro Compatible");
            k150_info.supports_icsp = 1;
            k150_info.max_rom_size = 16384;
            k150_info.max_eeprom_size = 256;
            printf("K150: ✓ Micropro.exe protocol detected\n");
            micropro_close();
            return PROTOCOL_MICROPRO;
        }
        micropro_close();
    }
    
    // Try standard P018 (fallback)
    printf("K150: Trying standard P018 protocol...\n");
    if (k150_open_port(port) == SUCCESS) {
        if (k150_detect_programmer() == SUCCESS) {
            k150_info.protocol = PROTOCOL_P018;
            k150_info.firmware_version = 0x03;
            strcpy(k150_info.version_string, "P018 Standard");
            k150_info.supports_icsp = 0;
            k150_info.max_rom_size = 8192;
            k150_info.max_eeprom_size = 256;
            printf("K150: ✓ P018 protocol detected\n");
            k150_close_port();
            return PROTOCOL_P018;
        }
        k150_close_port();
    }
    
    printf("K150: ✗ No compatible protocol detected\n");
    return PROTOCOL_UNKNOWN;
}

// Get protocol information
const K150_INFO* k150_get_info(void)
{
    return &k150_info;
}

// Print protocol information
void k150_print_info(void)
{
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║       K150 Programmer Information      ║\n");
    printf("╠════════════════════════════════════════╣\n");
    
    if (k150_info.protocol != PROTOCOL_UNKNOWN) {
        printf("║ Protocol:     %-24s ║\n", k150_info.version_string);
        printf("║ Firmware:     v%d.%d                     ║\n", 
               k150_info.firmware_version >> 4, k150_info.firmware_version & 0x0F);
        printf("║ ICSP Support: %-24s ║\n", k150_info.supports_icsp ? "Yes" : "No");
        printf("║ Max ROM:      %-24d ║\n", k150_info.max_rom_size);
        printf("║ Max EEPROM:   %-24d ║\n", k150_info.max_eeprom_size);
    } else {
        printf("║        No programmer detected         ║\n");
    }
    
    printf("╚════════════════════════════════════════╝\n\n");
}

// Unified programming function with protocol selection
int k150_program_auto(const char *port, const char *device_name, 
                      const unsigned char *rom_data, int rom_size, 
                      unsigned int config_word)
{
    // Auto-detect protocol if not already done
    if (k150_info.protocol == PROTOCOL_UNKNOWN) {
        if (k150_detect_protocol(port) == PROTOCOL_UNKNOWN) {
            fprintf(stderr, "K150: No compatible programmer detected\n");
            return ERROR;
        }
    }
    
    printf("K150: Using %s protocol for programming\n", k150_info.version_string);
    
    // Check ROM size limit
    if (rom_size > k150_info.max_rom_size) {
        fprintf(stderr, "K150: ROM size %d exceeds maximum %d for this protocol\n",
                rom_size, k150_info.max_rom_size);
        return ERROR;
    }
    
    int result = ERROR;
    
    // Use appropriate protocol
    switch (k150_info.protocol) {
        case PROTOCOL_P18A:
            printf("K150: Programming with P18A enhanced protocol\n");
            // P18A module temporarily disabled to avoid linking conflicts\n        if (k150_p18a_init(port) == SUCCESS) {
                result = k150_p18a_program_device(device_name, rom_data, rom_size, config_word);
                k150_p18a_close();
            }
            break;
            
        case PROTOCOL_MICROPRO:
            printf("K150: Programming with Micropro.exe protocol\n");
            if (micropro_open(port) == SUCCESS) {
                result = micropro_program_device(device_name, rom_data, rom_size, config_word);
                micropro_close();
            }
            break;
            
        case PROTOCOL_P018:
            printf("K150: Programming with standard P018 protocol\n");
            if (k150_open_port(port) == SUCCESS) {
                // Find device
                extern const PIC_DEFINITION *deviceArray[];
                const PIC_DEFINITION *device = NULL;
                for (int i = 0; deviceArray[i] != NULL; i++) {
                    if (strcasecmp(deviceArray[i]->name, device_name) == 0) {
                        device = deviceArray[i];
                        break;
                    }
                }
                
                if (device) {
                    // Use existing P018 functions
                    if (k150_erase_chip_enhanced(device) == SUCCESS) {
                        if (k150_program_rom_enhanced(device, rom_data, rom_size) == SUCCESS) {
                            // Write config
                            unsigned char cfg[2] = {config_word & 0xFF, (config_word >> 8) & 0xFF};
                            if (k150_program_config(cfg) == SUCCESS) {
                                result = k150_verify_rom_enhanced(device, rom_data, rom_size);
                            }
                        }
                    }
                } else {
                    fprintf(stderr, "K150: Unknown device: %s\n", device_name);
                }
                k150_close_port();
            }
            break;
            
        default:
            fprintf(stderr, "K150: Unsupported protocol\n");
            break;
    }
    
    return result;
}

// Protocol-specific erase function
int k150_erase_auto(const char *port, const char *device_name)
{
    if (k150_info.protocol == PROTOCOL_UNKNOWN) {
        if (k150_detect_protocol(port) == PROTOCOL_UNKNOWN) {
            return ERROR;
        }
    }
    
    printf("K150: Erasing %s using %s protocol\n", device_name, k150_info.version_string);
    
    switch (k150_info.protocol) {
        case PROTOCOL_P18A:
            // P18A module temporarily disabled to avoid linking conflicts\n        if (k150_p18a_init(port) == SUCCESS) {
                extern int k150_p18a_erase_chip(void);
                int result = k150_p18a_erase_chip();
                k150_p18a_close();
                return result;
            }
            break;
            
        case PROTOCOL_MICROPRO:
            if (micropro_open(port) == SUCCESS) {
                extern int micropro_init_sequence(void);
                extern int micropro_send_device_params(const char *);
                extern int micropro_erase_chip(void);
                
                if (micropro_init_sequence() == SUCCESS) {
                    if (micropro_send_device_params(device_name) == SUCCESS) {
                        int result = micropro_erase_chip();
                        micropro_close();
                        return result;
                    }
                }
                micropro_close();
            }
            break;
            
        case PROTOCOL_P018:
            // Use existing P018 erase
            return k150_erase_chip();
            
        default:
            break;
    }
    
    return ERROR;
}

// Protocol-specific read function
int k150_read_auto(const char *port, const char *device_name, 
                   unsigned char *buffer, int size)
{
    if (k150_info.protocol == PROTOCOL_UNKNOWN) {
        if (k150_detect_protocol(port) == PROTOCOL_UNKNOWN) {
            return ERROR;
        }
    }
    
    printf("K150: Reading %s using %s protocol\n", device_name, k150_info.version_string);
    
    switch (k150_info.protocol) {
        case PROTOCOL_P18A:
            // P18A module temporarily disabled to avoid linking conflicts\n        if (k150_p18a_init(port) == SUCCESS) {
                extern int k150_p18a_read_rom(unsigned char *, int, int);
                int result = k150_p18a_read_rom(buffer, 0, size);
                k150_p18a_close();
                return result;
            }
            break;
            
        case PROTOCOL_MICROPRO:
            if (micropro_open(port) == SUCCESS) {
                extern int micropro_init_sequence(void);
                extern int micropro_send_device_params(const char *);
                extern int micropro_read_block(unsigned int, unsigned char *, int);
                
                if (micropro_init_sequence() == SUCCESS) {
                    if (micropro_send_device_params(device_name) == SUCCESS) {
                        int result = SUCCESS;
                        int block_size = 64;
                        
                        for (int offset = 0; offset < size; offset += block_size) {
                            int chunk = (size - offset > block_size) ? block_size : (size - offset);
                            if (micropro_read_block(offset, buffer + offset, chunk) != SUCCESS) {
                                result = ERROR;
                                break;
                            }
                        }
                        
                        micropro_close();
                        return result;
                    }
                }
                micropro_close();
            }
            break;
            
        case PROTOCOL_P018:
            // Use existing P018 read
            return k150_read_rom(buffer, size);
            
        default:
            break;
    }
    
    return ERROR;
}

// Test all protocols
void k150_test_protocols(const char *port)
{
    printf("\n═══════════════════════════════════════\n");
    printf("    K150 Protocol Compatibility Test    \n");
    printf("═══════════════════════════════════════\n\n");
    
    // Test P18A
    printf("Testing P18A Enhanced Protocol...\n");
    if (k150_p18a_init(port) == SUCCESS) {
        printf("  ✓ P18A protocol supported\n");
        printf("  ✓ Advanced features available\n");
        k150_p18a_close();
    } else {
        printf("  ✗ P18A protocol not supported\n");
    }
    
    usleep(500000);
    
    // Test Micropro
    printf("\nTesting Micropro.exe Protocol...\n");
    if (micropro_open(port) == SUCCESS) {
        extern int micropro_init_sequence(void);
        if (micropro_init_sequence() == SUCCESS) {
            printf("  ✓ Micropro.exe protocol supported\n");
            printf("  ✓ Windows software compatible\n");
        } else {
            printf("  ✗ Micropro.exe handshake failed\n");
        }
        micropro_close();
    } else {
        printf("  ✗ Micropro.exe protocol not supported\n");
    }
    
    usleep(500000);
    
    // Test P018
    printf("\nTesting P018 Standard Protocol...\n");
    if (k150_open_port(port) == SUCCESS) {
        if (k150_detect_programmer() == SUCCESS) {
            printf("  ✓ P018 protocol supported\n");
            printf("  ✓ Basic programming available\n");
        } else {
            printf("  ✗ P018 handshake failed\n");
        }
        k150_close_port();
    } else {
        printf("  ✗ P018 protocol not supported\n");
    }
    
    printf("\n═══════════════════════════════════════\n");
    
    // Final detection
    K150_PROTOCOL detected = k150_detect_protocol(port);
    if (detected != PROTOCOL_UNKNOWN) {
        printf("\nRecommended protocol: %s\n", k150_info.version_string);
        k150_print_info();
    } else {
        printf("\n⚠ No compatible protocol found!\n");
        printf("Please check:\n");
        printf("  • USB cable connection\n");
        printf("  • Device permissions (sudo may be needed)\n");
        printf("  • K150 programmer firmware version\n");
    }
}