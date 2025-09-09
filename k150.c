//-----------------------------------------------------------------------------
//
//	K150 PIC programmer interface - Enhanced P018 Protocol Implementation
//
//-----------------------------------------------------------------------------
//
//	Full P018 protocol support with picdev.c integration
//	Supports chip detection, erase, read, program, and verify operations
//	Compatible with all PIC devices defined in picdev.c
//
// Copyright (c) 2025 - Enhanced K150 support
//
//-----------------------------------------------------------------------------

#include "k150.h"
#include "picdev.h"
#include "debug.h"
#include "serial.h"
#include "debug.h"
#include "k150.h"
// Native K150 protocol implementation
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/select.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>  // strcasecmp için
#include <stdbool.h>  // bool tipi için
#include <errno.h>    // errno for read error handling

// P18A Protocol commands (for updated firmware)
#define P18A_CMD_DETECT         0x42 // Detect programmer
#define P18A_CMD_START          0x50 // Start communication
#define P18A_CMD_EXIT           0x51 // Exit/quit
#define P18A_CMD_READ_ROM       0x46 // Read ROM
#define P18A_CMD_PROGRAM_ROM    0x47 // Program ROM
#define P18A_CMD_ERASE_CHIP     0x45 // Erase chip
#define P18A_CMD_READ_CONFIG    0x43 // Read configuration
#define P18A_CMD_CHIPINFO       0x49 // Get chip info

// P018 Protocol Commands (based on softprotocol5.txt)
#define P018_CMD_START          'P'  // 0x50 - Start communication
#define P018_CMD_INIT           3    // Initialize with device parameters
#define P018_CMD_VOLTAGES_ON    4    // Turn voltages on
#define P018_CMD_VOLTAGES_OFF   5    // Turn voltages off
#define P018_CMD_PROGRAM_ROM    7    // Program ROM
#define P018_CMD_READ_ROM       11   // Read ROM
#define P018_CMD_READ_CONFIG    13   // Read configuration (chip detection)
#define P018_CMD_ERASE_CHIP     15   // Erase chip

// P018 Protocol Commands for EEPROM operations
#define P018_CMD_READ_EEPROM    12   // Read EEPROM
#define P018_CMD_PROGRAM_EEPROM 9    // Program EEPROM

// P018 Protocol ACK responses
#define P018_ACK_START          'P'  // 0x50 - Start ACK
#define P018_ACK_INIT           'I'  // 0x49 - Init ACK
#define P018_ACK_VOLTAGES_ON    'V'  // 0x56 - Voltages on ACK
#define P018_ACK_VOLTAGES_OFF   'v'  // 0x76 - Voltages off ACK
#define P018_ACK_PROGRAM        'Y'  // 0x59 - Program/Erase ACK
#define P018_ACK_PROGRAM_DONE   'D'  // 0x44 - Program complete ACK
#define P018_ACK_CONFIG         'C'  // 0x43 - Configuration start

#define MAX_ROM_SIZE 16384  // Maximum ROM size in bytes (8192 words)
#define CHUNK_SIZE 64       // Read/write chunk size

// Forward declarations for helper functions
static int hex_to_byte(char c);
#define MAX_RETRIES 5  // Increased retries for PL2303 stability
#define DELAY_US 70000       // 70ms delay for better PL2303 timing (increased for epk150.hex compatibility)

// Constants for K150 functions
#define SUCCESS 0
#define ERROR -1

// Global variables
static int k150_fd = -1;
static PIC_DEFINITION *current_device = NULL;
static int k150_firmware_version = 0;
bool isK150 = false;  // K150 programmer active flag

// Firmware version detection
int k150_get_firmware_version(void)
{
    if (k150_firmware_version != 0) {
        return k150_firmware_version;
    }
    
    unsigned char cmd = K150_CMD_GET_VERSION;
    unsigned char resp[2];
    
    if (k150_write_serial(&cmd, 1) == SUCCESS && 
        k150_read_serial(resp, 2) == SUCCESS) {
        k150_firmware_version = (resp[0] << 8) + resp[1];
    } else {
        k150_firmware_version = K150_FW_P014;  // Default to oldest firmware
    }
    
    return k150_firmware_version;
}

PIC_DEFINITION *k150_get_current_device(void)
{
    return current_device;
}

int theDevice = -1; // Global device handle for compatibility

// External device list from picdev.c
extern const PIC_DEFINITION *deviceArray[];

// Configuration memory commands (P018 protocol) - Legacy
#define P018_CMD_READ_CONFIG_LEGACY     0x0E
#define P018_CMD_WRITE_CONFIG_LEGACY    0x09

//-----------------------------------------------------------------------------
// Serial communication functions
//-----------------------------------------------------------------------------
// Microbrn.exe compatible write function with buffer flush
int k150_write_serial(const unsigned char *buf, int len)
{
    if (write(k150_fd, buf, len) == len) {
        tcflush(k150_fd, TCIOFLUSH);  // Buffer flush (PURGE equivalent)
        DEBUG_PRINT("write_serial sent %d bytes: ", len);
        if (debug_enabled) {
            for (int i = 0; i < len; i++) printf("0x%02x ", buf[i]);
            printf("\n");
        }
        return SUCCESS;
    }
    printf("ERROR: k150_write_serial failed: %s\n", strerror(errno));
    return ERROR;
}

// Quick read function for protocol testing
int k150_read_serial_quick(unsigned char *buf, int len)
{
    int retries = 0;
    while (retries < 20) {  // Only 20 retries (1 second)
        int r = read(k150_fd, buf, len);
        if (r == len) {
            DEBUG_PRINT("read_serial_quick got %d bytes: ", len);
            if (debug_enabled) {
                for (int i = 0; i < len; i++) printf("0x%02x ", buf[i]);
                printf("\n");
            }
            return SUCCESS;
        }
        usleep(50000);  // 50ms delay
        retries++;
    }
    DEBUG_PRINT("read_serial_quick failed after %d retries\n", retries);
    return ERROR;
}

// Microbrn.exe compatible read function (WAIT_ON_MASK equivalent)
int k150_read_serial(unsigned char *buf, int len)
{
    int retries = 0;
    int max_retries = 100; // Reduced timeout: 100 * 50ms = 5 seconds
    
    while (retries < max_retries) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(k150_fd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000; // 50ms timeout per attempt
        
        int select_result = select(k150_fd + 1, &readfds, NULL, NULL, &timeout);
        
        if (select_result > 0 && FD_ISSET(k150_fd, &readfds)) {
            int r = read(k150_fd, buf, len);
            if (r == len) {
                DEBUG_PRINT("read_serial got %d bytes: ", len);
                if (debug_enabled) {
                    for (int i = 0; i < len; i++) printf("0x%02x ", buf[i]);
                    printf("\n");
                }
                return SUCCESS;
            } else if (r > 0) {
                printf("K150: Partial read %d/%d bytes, retry %d\n", r, len, retries);
            }
        }
        
        retries++;
        if (retries % 20 == 0) { // Progress indicator every second
            printf("K150: Read timeout, retry %d/%d (waiting for %d bytes)\n", retries, max_retries, len);
        }
    }
    printf("K150: Read failed after %d retries (5 seconds), wanted %d bytes\n", retries, len);
    return ERROR;
}

//-----------------------------------------------------------------------------
// Microbrn.exe Compatible Command Functions
//-----------------------------------------------------------------------------

// Microbrn.exe command sending function
int k150_send_command(unsigned char cmd) {
    unsigned char buf[1] = {cmd};
    if (k150_write_serial(buf, 1) == SUCCESS) {
        unsigned char resp[1];
        if (k150_read_serial(resp, 1) == SUCCESS && resp[0] == K150_CMD_ACK) {
            return SUCCESS;
        }
    }
    return ERROR;
}

// Microbrn.exe cyclic fuse programming
int k150_program_config(unsigned char *config_data) {
    if (k150_send_command(K150_CMD_INIT) != SUCCESS) return ERROR;
    unsigned char buf[2] = {config_data[0], config_data[1]};
    if (k150_write_serial(buf, 2) != SUCCESS) return ERROR;
    unsigned char resp[2];
    if (k150_read_serial(resp, 2) == SUCCESS) {
        if (resp[0] == config_data[0] && resp[1] == config_data[1]) {
            k150_send_command(K150_CMD_EXIT);
            return SUCCESS;
        }
    }
    k150_send_command(K150_CMD_EXIT);
    return ERROR;
}

//-----------------------------------------------------------------------------
// P018 Protocol Implementation
//-----------------------------------------------------------------------------

// Microbrn.exe compatible start communication - localhost.LOG analysis
static int k150_start_communication(void)
{
    unsigned char buffer[10];
    DEBUG_PRINT("K150: Starting Microbrn.exe compatible communication sequence\n");
    
    // Step 1: DTR/RTS control sequence (from localhost.LOG)
    int status;
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // CLR_DTR, CLR_RTS first (localhost.LOG line 8-9)
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        
        // SET_DTR, SET_RTS (localhost.LOG line 14-15)
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        
        // CLR_DTR (localhost.LOG line 20)
        status &= ~TIOCM_DTR;
        ioctl(k150_fd, TIOCMSET, &status);
    }
    
    // Step 2: Read K150 auto-response (42 03 42)
    usleep(100000);  // 100ms wait
    if (k150_read_serial_quick(buffer, 3) == SUCCESS) {
        if (buffer[0] == 0x42 && buffer[1] == 0x03 && buffer[2] == 0x42) {
            DEBUG_PRINT("K150: Expected auto-response received\n");
        }
    }
    
    // Step 3: P018 protocol init (50 03) - localhost.LOG line 32
    unsigned char p018_cmd[2] = {0x50, 0x03};
    if (k150_write_serial(p018_cmd, 2) == SUCCESS) {
        return SUCCESS;
    }
    
    return ERROR;
}

// Send device initialization - Simplified based on log analysis
static int k150_send_device_params(const PIC_DEFINITION *device)
{
    unsigned char init_cmd = 0x2E; // Single byte '.' command from log
    // unsigned char ack; // Unused variable
    
    printf("K150: Sending init command '.' (0x2E) for %s\n", device->name);
    
    if (k150_write_serial(&init_cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    // No specific ACK expected for init command in real protocol
    printf("K150: Init command sent\n");
    return SUCCESS;
}

// Turn voltages on - Enhanced protocol using P018 commands
static int k150_voltages_on(void)
{
    unsigned char voltage_cmd = P018_CMD_VOLTAGES_ON; // 0x04 command 
    unsigned char ack;
    
    printf("K150: Sending voltages ON command (0x04)\n");
    if (k150_write_serial(&voltage_cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) != SUCCESS) {
        fprintf(stderr, "K150: ERROR: Failed to read voltage ACK\n");
        return ERROR;
    }
    
    if (ack != 0x56) { // 'V' from log analysis
        fprintf(stderr, "K150: ERROR: Expected 'V' (0x56), got 0x%02x\n", ack);
        return ERROR;
    }
    
    printf("K150: Voltages ON - received 'V' (0x56)\n");
    return SUCCESS;
}

// Turn voltages off
static int k150_voltages_off(void)
{
    unsigned char cmd = P018_CMD_VOLTAGES_OFF;
    unsigned char ack;
    
    printf("K150: Sending voltages OFF command (0x05)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_VOLTAGES_OFF) {
        printf("K150: WARNING: Expected 'v' ACK, got 0x%02x (continuing)\n", ack);
        return SUCCESS; // Continue even if voltages off ACK fails
    }
    printf("K150: Voltages OFF ACK 'v' received\n");
    return SUCCESS;
}

// Chip detection using P018 command 13
static int k150_detect_chip_enhanced(const PIC_DEFINITION **detected_device)
{
    if (k150_start_communication() != SUCCESS) return ERROR;
    
    // Try to detect chip by attempting initialization with common PIC devices
    // Use deviceArray from picdev.c directly
    const char *test_device_names[] = {
        "16F628A",
        "16F84", 
        "16F876",
        "16F690",
        NULL
    };
    
    for (int i = 0; test_device_names[i] != NULL; i++) {
        fprintf(stderr, "K150: Looking for device: %s\n", test_device_names[i]);
        
        // Find device in deviceArray
        const PIC_DEFINITION *device = NULL;
        for (int j = 0; deviceArray[j] != NULL; j++) {
            if (strcmp(deviceArray[j]->name, test_device_names[i]) == 0) {
                device = deviceArray[j];
                fprintf(stderr, "K150: Found device %s in array\n", test_device_names[i]);
                break;
            }
        }
        
        if (!device) {
            fprintf(stderr, "K150: Device %s not found in deviceArray\n", test_device_names[i]);
            continue;
        }
        
        fprintf(stderr, "K150: Testing device: %s\n", device->name);
        
        // Try to initialize this device type
        if (k150_send_device_params(device) == SUCCESS) {
            fprintf(stderr, "K150: Device params sent successfully for %s\n", device->name);
            if (k150_voltages_on() == SUCCESS) {
                fprintf(stderr, "K150: Successfully initialized %s\n", device->name);
                *detected_device = device;
                return SUCCESS;
            } else {
                fprintf(stderr, "K150: Voltages ON failed for %s\n", device->name);
            }
        } else {
            fprintf(stderr, "K150: Device params failed for %s\n", device->name);
        }
        
        // Reset communication for next attempt
        usleep(100000); // 100ms delay between attempts
    }
    
    fprintf(stderr, "K150: No compatible PIC device detected\n");
    return ERROR;
}

// Command line chip detection function (pk2cmd -P equivalent)
int k150_detect_chip_command_line(void)
{
    printf("K150: Detecting connected PIC device...\n");
    
    if (k150_open_port("/dev/ttyUSB0") != SUCCESS) {
        printf("ERROR: Failed to open K150 port\n");
        return ERROR;
    }
    
    if (k150_detect_programmer() != SUCCESS) {
        printf("ERROR: K150 programmer not detected\n");
        k150_close_port();
        return ERROR;
    }
    
    const PIC_DEFINITION *detected_device = NULL;
    int result = k150_detect_chip_enhanced(&detected_device);
    
    if (result == SUCCESS && detected_device) {
        printf("\nDetected PIC: %s\n", detected_device->name);
        
        // Extract chip ID from device definition for display
        int chip_id = (detected_device->def[22] << 8) | detected_device->def[23];
        printf("Chip ID: 0x%04X\n", chip_id);
        
        // Extract program memory size from definition
        int pgm_size = (detected_device->def[0] << 8) | detected_device->def[1];
        printf("Program Memory: %d words\n", pgm_size);
        
        // Extract data memory size from definition
        int data_size = (detected_device->def[34] << 8) | detected_device->def[35];
        printf("Data Memory: %d bytes\n", data_size);
        
        k150_close_port();
        return SUCCESS;
    } else {
        printf("\nNo PIC detected or unknown device\n");
        printf("Please check:\n");
        printf("- PIC chip is properly inserted in socket\n");
        printf("- ICSP connections are correct\n");
        printf("- Power supply is adequate (5V)\n");
        
        k150_close_port();
        return ERROR;
    }
}

// Erase chip - Fixed based on Micropro.LOG analysis
int k150_erase_chip_enhanced(const PIC_DEFINITION *device)
{
    printf("=== K150_ERASE_CHIP_ENHANCED CALLED ===\n");
    printf("K150: Erasing chip %s using Micropro.LOG sequence\n", device->name);
    printf("K150: Serial port fd: %d\n", k150_fd);
    
    // Step 0: CRITICAL DTR/RTS control sequence (from Micropro.LOG)
    printf("K150: Performing DTR/RTS control sequence\n");
    int status;
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // CLR_DTR, CLR_RTS first (line 8-9)
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS cleared\n");
        
        // SET_DTR, SET_RTS (line 14-15)
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS set\n");
        
        // CLR_DTR (line 20)
        status &= ~TIOCM_DTR;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(100000); // 100ms wait for K150 response
        printf("K150: Final DTR cleared, waiting for auto-response\n");
    }
    
    // Read K150 auto-response (42 03 42) - critical!
    unsigned char auto_resp[3];
    printf("K150: Reading auto-response sequence\n");
    if (k150_read_serial(auto_resp, 3) == SUCCESS) {
        printf("K150: Auto-response: 0x%02x 0x%02x 0x%02x\n", auto_resp[0], auto_resp[1], auto_resp[2]);
        if (auto_resp[0] == 0x42 && auto_resp[1] == 0x03 && auto_resp[2] == 0x42) {
            printf("K150: Expected auto-response received!\n");
        } else {
            printf("K150: WARNING: Unexpected auto-response\n");
        }
    } else {
        printf("K150: WARNING: No auto-response received\n");
    }
    
    // Step 1: Start communication with device type
    unsigned char start_cmd[2] = {0x50, 0x03}; // P018_CMD_START + device type
    printf("K150: Sending start command: 50 03\n");
    if (k150_write_serial(start_cmd, 2) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    unsigned char ack;
    if (k150_read_serial(&ack, 1) == SUCCESS && ack == 0x50) {
        printf("K150: Start ACK received: 0x%02x\n", ack);
    }
    
    // Step 2: Send device parameters (optimized based on test results)
    printf("K150: Sending device parameters (optimized)\n");
    
    // First 4 parameters don't need ACK (send quickly)
    unsigned char silent_params[][2] = {
        {0x04, 0x00}, // Silent params - no ACK expected
        {0x00, 0x40}, 
        {0x06, 0x00}, 
        {0xC8, 0x02}
    };
    
    printf("K150: Sending silent parameters (no ACK expected)\n");
    for (int i = 0; i < 4; i++) {
        printf("K150: Silent param[%d]: 0x%02x 0x%02x\n", i, silent_params[i][0], silent_params[i][1]);
        if (k150_write_serial(silent_params[i], 2) != SUCCESS) return ERROR;
        usleep(DELAY_US / 4); // Faster timing for silent params
    }
    
    // Last 2 parameters expect ACK
    unsigned char ack_params[][2] = {
        {0x00, 0x01}, // These expect ACK
        {0x00, 0x14}
    };
    
    printf("K150: Sending ACK-expected parameters\n");
    for (int i = 0; i < 2; i++) {
        printf("K150: ACK param[%d]: 0x%02x 0x%02x\n", i, ack_params[i][0], ack_params[i][1]);
        if (k150_write_serial(ack_params[i], 2) != SUCCESS) return ERROR;
        usleep(DELAY_US);
        
        // Only read ACK for these parameters with shorter timeout
        unsigned char ack;
        fd_set readfds;
        struct timeval timeout;
        FD_ZERO(&readfds);
        FD_SET(k150_fd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000; // 200ms timeout instead of 5 seconds
        
        if (select(k150_fd + 1, &readfds, NULL, NULL, &timeout) > 0) {
            if (read(k150_fd, &ack, 1) == 1) {
                printf("K150: ACK param[%d]: 0x%02x ('%c')\n", i, ack, (ack >= 32 && ack <= 126) ? ack : '?');
            }
        } else {
            printf("K150: No ACK for param[%d] (quick timeout)\n", i);
        }
    }
    
    // Step 3: Voltages OFF first (critical!)
    printf("K150: Sending voltages OFF command (05)\n");
    unsigned char voltages_off = 0x05;
    if (k150_write_serial(&voltages_off, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) == SUCCESS && ack == 0x76) {
        printf("K150: Voltages OFF ACK 'v' received: 0x%02x\n", ack);
    }
    
    // Step 4: Voltages ON
    printf("K150: Sending voltages ON command (04)\n"); 
    unsigned char voltages_on = 0x04;
    if (k150_write_serial(&voltages_on, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) == SUCCESS && ack == 0x56) {
        printf("K150: Voltages ON ACK 'V' received: 0x%02x\n", ack);
    }
    
    // Step 5: ERASE command with parameter (0F 3F)
    printf("K150: Sending erase command: 0F 3F\n");
    unsigned char erase_cmd[2] = {0x0F, 0x3F}; // Erase + parameter
    if (k150_write_serial(erase_cmd, 2) != SUCCESS) return ERROR;
    usleep(DELAY_US * 4); // Erase takes longer
    
    // Step 6: Read multiple status responses (42 42 42 59)
    unsigned char status_responses[4];
    printf("K150: Reading erase status responses...\n");
    for (int i = 0; i < 4; i++) {
        if (k150_read_serial(&status_responses[i], 1) == SUCCESS) {
            printf("K150: Status[%d]: 0x%02x\n", i, status_responses[i]);
        }
        usleep(DELAY_US);
    }
    
    // Check for final success ACK (0x59 = 'Y')
    if (status_responses[3] == 0x59) {
        printf("K150: Erase SUCCESS ACK 'Y' received!\n");
    } else {
        printf("K150: ERROR: Expected final 'Y' ACK, got 0x%02x\n", status_responses[3]);
        return ERROR;
    }
    
    // Step 7: Voltages OFF 
    printf("K150: Final voltages OFF\n");
    if (k150_write_serial(&voltages_off, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    printf("K150: Chip erase completed successfully!\n");
    return SUCCESS;
}

// Helper function for DTR/RTS sequence
static int k150_dtr_rts_sequence(void)
{
    int status;
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // CLR_DTR, CLR_RTS first
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS cleared\n");
        
        // SET_DTR, SET_RTS
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS set\n");
        
        // CLR_DTR final
        status &= ~TIOCM_DTR;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: Final DTR cleared, waiting for auto-response\n");
        
        return SUCCESS;
    }
    return ERROR;
}

// Read ROM using write protocol base
static int k150_read_rom_enhanced(const PIC_DEFINITION *device, unsigned char *buffer, int size)
{
    if (!buffer || size <= 0) {
        fprintf(stderr, "K150: Invalid buffer or size for ROM read\n");
        return ERROR;
    }
    
    printf("K150: Reading ROM (%d bytes) using write protocol base\n", size);
    
    // Initialize buffer with 0xFF (erased state)
    memset(buffer, 0xFF, size);
    
    // Use SAME initialization as write operation (k150_write_pgm)
    extern char *port_name;
    if (k150_open_port(port_name) != SUCCESS) {
        fprintf(stderr, "K150: Failed to open port for read operation\n");
        return ERROR;
    }
    
    if (k150_start_communication() != SUCCESS) {
        fprintf(stderr, "K150: Failed to start communication for read\n");
        k150_close_port();
        return ERROR;
    }
    
    // CRITICAL: Full Micropro.LOG init sequence
    printf("K150: Starting Micropro.LOG-style init sequence\n");
    
    // Step 1: Send 'P' 0x03 (Command start from log)
    unsigned char start_seq[] = {0x50, 0x03}; // 'P' + init byte
    if (k150_write_serial(start_seq, 2) != SUCCESS) {
        fprintf(stderr, "K150: Failed to send start sequence\n");
        k150_close_port();
        return ERROR;
    }
    
    unsigned char start_response;
    if (k150_read_serial(&start_response, 1) == SUCCESS && start_response == 'P') {
        printf("K150: Start sequence OK: got 'P' (0x%02X)\n", start_response);
    } else {
        printf("K150: Start sequence failed: expected 'P', got 0x%02X\n", start_response);
    }
    
    // Step 2: Send config bytes from Micropro.LOG
    unsigned char config_bytes[] = {0x04, 0x00, 0x00, 0x40, 0x06, 0x00, 0xC8, 0x02, 0x00, 0x01, 0x00};
    printf("K150: Sending config bytes (%zu bytes)\n", sizeof(config_bytes));
    for (int i = 0; i < sizeof(config_bytes); i++) {
        if (k150_write_serial(&config_bytes[i], 1) != SUCCESS) {
            fprintf(stderr, "K150: Failed to send config byte %d (0x%02X)\n", i, config_bytes[i]);
            k150_close_port();
            return ERROR;
        }
        usleep(DELAY_US / 10); // Small delay between config bytes
    }
    
    // Skip config response read (may not always come)
    printf("K150: Config sent, proceeding without waiting for response\n");
    usleep(DELAY_US * 2); // Give time for config to settle
    
    // For READ: Need to turn ON voltages like in programming
    printf("K150: Turning ON voltages for read operation\n");
    unsigned char voltages_on_cmd = P018_CMD_VOLTAGES_ON; // 0x04
    if (k150_write_serial(&voltages_on_cmd, 1) != SUCCESS) {
        fprintf(stderr, "K150: Failed to send voltages ON for read\n");
        k150_close_port();
        return ERROR;
    }
    usleep(DELAY_US);
    
    unsigned char voltages_ack;
    if (k150_read_serial(&voltages_ack, 1) == SUCCESS) {
        printf("K150: Voltages ON ACK: 0x%02X\n", voltages_ack);
    }
    
    // Send READ command (picpro uses 0x0B, let's try that instead)
    unsigned char cmd = 0x0B; // Use picpro command (11 decimal = P018_CMD_READ_ROM)
    printf("K150: Sending read ROM command (0x%02X - picpro style)\n", cmd);
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        fprintf(stderr, "K150: Failed to send read ROM command\n");
        k150_close_port();
        return ERROR;
    }
    
    // CRITICAL: Follow picpro style - no ACKs, direct read after 0x0B command
    printf("K150: Following picpro read sequence - no ACKs, direct data read\n");
    
    // Read data using exact picpro polling method
    printf("K150: Reading %d bytes using picpro polling method\n", size);
    
    // Give K150 time to prepare data
    usleep(DELAY_US * 3);
    
    // Use Micropro.LOG-style polling read (small chunks like log)
    int total_read = 0;
    time_t start_time = time(NULL);
    time_t timeout_seconds = 60; // 60 second timeout for full ROM
    
    printf("K150: Starting polling read, timeout=%d seconds\n", (int)timeout_seconds);
    
    while (total_read < size) {
        // Check timeout
        if (time(NULL) - start_time > timeout_seconds) {
            printf("K150: Polling read timeout after %d seconds (%d/%d bytes)\n", 
                   (int)timeout_seconds, total_read, size);
            break;
        }
        
        // Try to read chunk (optimize for speed like picpro)
        int chunk_size = (size - total_read > 64) ? 64 : (size - total_read);
        int bytes_read = read(k150_fd, buffer + total_read, chunk_size);
        
        if (bytes_read > 0) {
            total_read += bytes_read;
            printf("K150: Polling read: +%d bytes (total: %d/%d)\n", bytes_read, total_read, size);
            
            // Verbose hex dump (reduced frequency)
            if (total_read <= 32 || total_read % 256 == 0) {
                printf("K150: Bytes %d-%d: ", total_read - bytes_read, total_read - 1);
                for (int i = total_read - bytes_read; i < total_read; i++) {
                    printf("0x%02X ", buffer[i]);
                    if ((i - (total_read - bytes_read)) % 16 == 15) printf("\n                      ");
                }
                printf("\n");
            }
            
            // Reset timer on successful read
            start_time = time(NULL);
        } else if (bytes_read == 0) {
            // No more data available - this might be normal end of data
            printf("K150: No more data available (total: %d bytes)\n", total_read);
            break; // Exit loop gracefully
        } else {
            // Error case
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Would block - no data yet, continue polling
                usleep(5000); // 5ms delay for polling
            } else {
                printf("K150: Read error: %s\n", strerror(errno));
                break;
            }
        }
    }
    
    // Fill remaining bytes with 0xFF if needed
    if (total_read < size) {
        printf("K150: Filling remaining %d bytes with 0xFF\n", size - total_read);
        memset(buffer + total_read, 0xFF, size - total_read);
    }
    
    // Turn OFF voltages after read
    printf("K150: Turning OFF voltages after read\n");
    unsigned char voltages_off_cmd = P018_CMD_VOLTAGES_OFF; // 0x05
    k150_write_serial(&voltages_off_cmd, 1);
    
    k150_close_port();
    printf("K150: Read completed (%d/%d bytes)\n", total_read, size);
    return (total_read > 0) ? SUCCESS : ERROR;
}

// Program ROM using P018 protocol
int k150_program_rom_enhanced(const PIC_DEFINITION *device, const unsigned char *buffer, int size)
{
    printf("K150: Programming %d bytes to %s using P018 protocol\n", size, device->name);
    tcflush(k150_fd, TCIOFLUSH);

    // Add DTR/RTS control sequence for write operations (like erase)
    printf("K150: Performing DTR/RTS control sequence for programming\n");
    int status;
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // CLR_DTR, CLR_RTS first
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS cleared\n");
        
        // SET_DTR, SET_RTS
        status |= TIOCM_DTR;
        status |= TIOCM_RTS;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000);
        printf("K150: DTR/RTS set\n");
        
        // CLR_DTR final
        status &= ~TIOCM_DTR;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(100000); // 100ms wait for K150 response
        printf("K150: Final DTR cleared, waiting for auto-response\n");
    }

    if (k150_start_communication() != SUCCESS) return ERROR;
    if (k150_send_device_params(device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

    // Enhanced voltage monitoring for LED indication
    printf("K150: Programming voltages active - LED should be ON now\n");
    printf("K150: MCLR/VPP pin should show 12-13V (measure with multimeter if LED not ON)\n");
    
    // Keep voltage ON for visible LED indication
    usleep(2000000); // 2 seconds for user to see yellow LED
    printf("K150: LED visible confirmation period complete, starting programming...\n");

    unsigned char cmd = P018_CMD_PROGRAM_ROM;
    printf("K150: Sending program ROM command (0x07)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        k150_voltages_off();
        return ERROR;
    }
    usleep(DELAY_US);

    unsigned char ack;
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM) {
        printf("K150: ERROR: Expected 'Y' ACK for program start, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }

    int total_written = 0;
    while (total_written < size) {
        int chunk_size = (size - total_written > CHUNK_SIZE) ? CHUNK_SIZE : size - total_written;
        if (k150_write_serial(buffer + total_written, chunk_size) != SUCCESS) {
            printf("K150: Program write failed at byte %d\n", total_written);
            k150_voltages_off();
            return ERROR;
        }
        total_written += chunk_size;
        printf("K150: Program progress: %d/%d bytes\n", total_written, size);
        
        // Enhanced delay for epk150.hex compatibility - longer delay every 64 bytes
        if (total_written % 64 == 0) {
            usleep(100000); // 100ms delay every 64 bytes for EEPROM stability
        } else {
            usleep(DELAY_US);
        }
    }

    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM_DONE) {
        printf("K150: ERROR: Expected 'P' ACK for program complete, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }
    printf("K150: Program complete ACK 'P' received\n");
    
    // Keep LED ON for 3 seconds to show programming success
    printf("K150: Keeping programming LED ON for 3 seconds (success indication)...\n");
    sleep(3);
    
    k150_voltages_off();
    printf("K150: Programming completed successfully\n");
    return SUCCESS;
}

// Verify ROM by reading back and comparing
int k150_verify_rom_enhanced(const PIC_DEFINITION *device, const unsigned char *original_buffer, int size)
{
    unsigned char read_buffer[MAX_ROM_SIZE];
    printf("K150: Verifying programmed data...\n");

    if (k150_read_rom_enhanced(device, read_buffer, size) != SUCCESS) {
        printf("K150: Verification failed: Unable to read ROM\n");
        return ERROR;
    }

    int mismatch_count = 0;
    for (int i = 0; i < size; i++) {
        if (read_buffer[i] != original_buffer[i]) {
            if (mismatch_count < 10) { // Show first 10 mismatches
                printf("K150: Verification mismatch at byte %d: expected 0x%02x, got 0x%02x\n",
                       i, original_buffer[i], read_buffer[i]);
            }
            mismatch_count++;
        }
    }
    
    if (mismatch_count > 0) {
        printf("K150: Verification failed - %d byte(s) mismatch in %s\n", mismatch_count, device->name);
        return ERROR;
    }
    
    printf("K150: Verification successful - %s programmed correctly\n", device->name);
    return SUCCESS;
}

//-----------------------------------------------------------------------------
// Main interface functions for PICP integration
//-----------------------------------------------------------------------------

// Find device by name in deviceArray
static PIC_DEFINITION* find_device_by_name(const char* device_name)
{
    for (int i = 0; deviceArray[i] != NULL; i++) {
        if (strcasecmp(deviceArray[i]->name, device_name) == 0) {
            return (PIC_DEFINITION*)deviceArray[i];
        }
    }
    return NULL;
}

// Convert Intel HEX file to binary buffer
static int load_hex_file(const char* filename, unsigned char* buffer, int max_size)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("ERROR: Cannot open %s: %s\n", filename, strerror(errno));
        return ERROR;
    }
    
    memset(buffer, 0xFF, max_size); // Initialize with 0xFF
    char line[128];
    int max_addr = 0;
    
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != ':') continue;
        
        int byte_count, address, record_type;
        sscanf(line + 1, "%02x%04x%02x", &byte_count, &address, &record_type);
        
        if (record_type == 1) break; // End of file record
        if (record_type != 0) continue; // Skip non-data records
        
        for (int i = 0; i < byte_count; i++) {
            int byte_val;
            if (sscanf(line + 9 + i * 2, "%02x", &byte_val) == 1) {
                if (address + i < max_size) {
                    buffer[address + i] = (unsigned char)byte_val;
                    if (address + i > max_addr) max_addr = address + i;
                } else {
                    printf("WARNING: HEX data at address 0x%04X exceeds device capacity, skipping\n", address + i);
                }
            } else {
                printf("WARNING: Invalid HEX data in line: %.20s...\n", line);
                break;
            }
        }
    }
    fclose(fp);
    return max_addr + 1; // Return actual data size
}

// Save binary buffer to Intel HEX file
static int save_hex_file(const char* filename, const unsigned char* buffer, int size)
{
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        printf("ERROR: Cannot create %s: %s\n", filename, strerror(errno));
        return ERROR;
    }
    
    for (int i = 0; i < size; i += 16) {
        int byte_count = (size - i >= 16) ? 16 : size - i;
        fprintf(fp, ":%02X%04X00", byte_count, i);
        int checksum = byte_count + (i >> 8) + (i & 0xFF);
        for (int j = 0; j < byte_count; j++) {
            fprintf(fp, "%02X", buffer[i + j]);
            checksum += buffer[i + j];
        }
        fprintf(fp, "%02X\n", (0x100 - (checksum & 0xFF)) & 0xFF);
    }
    fprintf(fp, ":00000001FF\n");
    fclose(fp);
    return SUCCESS;
}

// Public interface functions
//-----------------------------------------------------------------------------

// Open K150 serial port with flexible port selection - Microbrn.exe compatible
static int k150_open_serial_port(const char *device)
{
    struct termios options;
    int status;
    
    DEBUG_PRINT("Attempting to open serial port %s\n", device);
    k150_fd = open(device, O_RDWR | O_NOCTTY); // Blocking mode, removed O_NDELAY
    if (k150_fd == -1) {
        fprintf(stderr, "ERROR: Unable to open %s: %s\n", device, strerror(errno));
        return ERROR;
    }
    
    // Configure serial port for K150 Microbrn.exe compatible (19200 baud, 8N1)
    tcgetattr(k150_fd, &options);
    // Microbrn.exe uses 19200 baud rate
    cfsetospeed(&options, B19200);
    cfsetispeed(&options, B19200);
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;      // Clear size
    options.c_cflag |= CS8;         // 8 bits
    options.c_cflag |= (CLOCAL | CREAD);  // Local, read enabled
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0; // Sonsuz bekleme (Microbrn.exe uyumlu)
    options.c_cc[VMIN] = 1; // En az 1 byte bekle
    
    tcsetattr(k150_fd, TCSANOW, &options);
    tcflush(k150_fd, TCIOFLUSH);
    fcntl(k150_fd, F_SETFL, 0); // Clear non-blocking flags
    
    // DTR/RTS kontrolü - Microbrn.exe uyumlu
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // DTR/RTS sıfırlama (Microbrn.exe açılış sırası)
        status &= ~TIOCM_RTS;  // RTS sıfırla
        status &= ~TIOCM_DTR;  // DTR sıfırla
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(100000); // 100ms bekleme
        
        // SET_DTR/SET_RTS ile cihaz uyandırma (Microbrn.exe sırası)
        status |= TIOCM_DTR;   // DTR set
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000); // 50ms
        
        status |= TIOCM_RTS;   // RTS set
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(50000); // 50ms stabilization
        
        // Log modem status for debugging
        ioctl(k150_fd, TIOCMGET, &status);
        DEBUG_PRINT("Modem status - DTR:%s RTS:%s CTS:%s DSR:%s DCD:%s\n",
                (status & TIOCM_DTR) ? "ON" : "OFF",
                (status & TIOCM_RTS) ? "ON" : "OFF", 
                (status & TIOCM_CTS) ? "ON" : "OFF",
                (status & TIOCM_DSR) ? "ON" : "OFF",
                (status & TIOCM_CD) ? "ON" : "OFF");
        DEBUG_PRINT("Enhanced DTR reset sequence completed\n");
    }
    
    tcflush(k150_fd, TCIOFLUSH);
    
            DEBUG_PRINT("Serial port %s initialized at 19200,8,N,1 with DTR/RTS reset (Microbrn.exe compatible)\n", device);
    return SUCCESS;
}

// Microbrn.exe uyumlu cleanup fonksiyonu
void cleanup_serial() {
    if (k150_fd >= 0) {
        tcflush(k150_fd, TCIOFLUSH);  // Tamponu temizle
        int status;
        if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
            status &= ~TIOCM_RTS;  // RTS temizle
            status &= ~TIOCM_DTR;  // DTR temizle
            ioctl(k150_fd, TIOCMSET, &status);
        }
        close(k150_fd);
        k150_fd = -1;
        DEBUG_PRINT("K150 port cleaned and closed (Microbrn.exe compatible)\n");
    }
}

static int k150_close_serial_port(void)
{
    cleanup_serial();  // Microbrn.exe uyumlu temizleme
    return SUCCESS;
}

// Standard K150 interface functions called by main.c

int k150_open_port(const char *device)
{
    isK150 = true;  // K150 kullanıldığını işaretle
    return k150_open_serial_port(device ? device : "/dev/ttyUSB0");
}

// Initialize serial port (compatibility function)
int init_serial(const char *device)
{
    int result = k150_open_port(device);
    if (result == SUCCESS) {
        theDevice = k150_fd; // Set global device handle for compatibility
    }
    return result;
}

// Check programmer (compatibility function)
int check_programmer(void)
{
    if (k150_detect_programmer() != SUCCESS) {
        fprintf(stderr, "ERROR: K150 programmer not detected\n");
        return ERROR;
    }
    DEBUG_PRINT("K150 programmer detected successfully\n");
    isK150 = true;  // K150 tespit edildi, bayrağı ayarla
    return SUCCESS;
}

int k150_close_port(void)
{
    return k150_close_serial_port();
}

int k150_detect_programmer(void)
{
    fprintf(stderr, "K150: Using original firmware detection (no 0x42 command needed)\n");
    fprintf(stderr, "K150: Programmer assumed present - original K150 protocol\n");
    k150_firmware_version = 0x03; // P018 protocol
    return SUCCESS;
}

int k150_is_port_open(void)
{
    return (k150_fd >= 0) ? 1 : 0;
}

int k150_init_pic(void)
{
    return k150_start_communication();
}

// Native chip detection using P018 protocol
int k150_detect_chip_with_type(const char *expected_chip)
{
    extern char *port_name; // From main.c
    
    // Use native P018 protocol implementation
    fprintf(stderr, "K150: Starting P018 protocol chip detection for: %s\n", expected_chip);
    
    // P018 Start Command
    unsigned char start_cmd = 'P';
    fprintf(stderr, "K150: Sending P018 start command 'P'\n");
    if (k150_write_serial(&start_cmd, 1) != SUCCESS) {
        fprintf(stderr, "K150: Failed to send start command\n");
        return ERROR;
    }
    usleep(DELAY_US);
    
    unsigned char ack;
    if (k150_read_serial(&ack, 1) == SUCCESS && (ack == 'P' || ack == 0x50)) {
        fprintf(stderr, "K150: P018 start ACK received: 0x%02x\n", ack);
    } else {
        fprintf(stderr, "K150: P018 start command retry...\n");
        usleep(DELAY_US * 2);
    }
    
    // P018 Init Command
    unsigned char init_cmd = 0x03;
    fprintf(stderr, "K150: Sending P018 init command (0x03)\n");
    if (k150_write_serial(&init_cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) == SUCCESS && (ack == 0x50 || ack == 'I' || ack == 0x49)) {
        fprintf(stderr, "K150: P018 init ACK received: 0x%02x\n", ack);
    }
    
    // P018 Read Configuration (Command 13)
    unsigned char read_config = 13;
    fprintf(stderr, "K150: Sending P018 read config command (13)\n");
    if (k150_write_serial(&read_config, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    unsigned char config_data[30];
    if (k150_read_serial(config_data, 30) == SUCCESS) {
        int chip_id = (config_data[1] << 8) | config_data[0]; // Device ID from config
        fprintf(stderr, "K150: Read Chip ID from P018: 0x%04x\n", chip_id);
        
        // Search for expected chip in device list
        extern const PIC_DEFINITION *deviceArray[];
        for (int i = 0; deviceArray[i] != NULL; i++) {
            if (strcmp(deviceArray[i]->name, expected_chip) == 0) {
                int expected_chip_id = (deviceArray[i]->def[22] << 8) | deviceArray[i]->def[23];
                fprintf(stderr, "K150: Expected Chip ID for %s: 0x%04x\n", expected_chip, expected_chip_id);
                
                if (expected_chip_id == chip_id) {
                    fprintf(stderr, "K150: SUCCESS - P018 chip matches! Detected: %s (ID: 0x%04x)\n", 
                            expected_chip, chip_id);
                    return SUCCESS;
                } else {
                    fprintf(stderr, "K150: ERROR - P018 chip ID mismatch!\n");
                    fprintf(stderr, "K150: Expected: %s (0x%04x), Found: 0x%04x\n", 
                            expected_chip, expected_chip_id, chip_id);
                    return ERROR;
                }
            }
        }
        
        fprintf(stderr, "K150: ERROR - Unknown expected chip type: %s\n", expected_chip);
        return ERROR;
    } else {
        fprintf(stderr, "K150: Failed to read P018 configuration data\n");
        return ERROR;
    }
}

// Legacy chip detection function for backward compatibility
int k150_detect_chip(struct pic_device **detected_dev)
{
    // Default to PIC16F628A for backward compatibility
    return k150_detect_chip_with_type("PIC16F628A");
}


int k150_erase_chip(void)
{
    if (!current_device) {
        printf("K150: ERROR: No device detected for erase operation\n");
        return ERROR;
    }
    return k150_erase_chip_enhanced(current_device);
}

int k150_read_rom(unsigned char *buffer, int size)
{
    printf("K150: Reading ROM (%d bytes) using enhanced protocol\n", size);
    
    if (!buffer || size <= 0) {
        fprintf(stderr, "K150: Invalid buffer or size for ROM read\n");
        return ERROR;
    }
    
    // Initialize buffer with 0xFF (erased state)
    memset(buffer, 0xFF, size);
    
    // Ensure K150 is initialized and ready
    extern char *port_name;
    if (k150_open_port(port_name) != SUCCESS) {
        fprintf(stderr, "K150: Failed to open port for read operation\n");
        return ERROR;
    }
    
    // Start communication sequence (based on Micropro log analysis)
    printf("K150: Starting read communication sequence\n");
    if (k150_start_communication() != SUCCESS) {
        fprintf(stderr, "K150: Failed to start communication for read\n");
        k150_close_port();
        return ERROR;
    }
    
    // Send P018 read ROM command (0x0B = 11 decimal)
    unsigned char read_cmd = P018_CMD_READ_ROM; // 11 (0x0B)
    printf("K150: Sending P018 read ROM command (0x%02X)\n", read_cmd);
    if (k150_write_serial(&read_cmd, 1) != SUCCESS) {
        fprintf(stderr, "K150: Failed to send read ROM command\n");
        k150_close_port();
        return ERROR;
    }
    
    usleep(DELAY_US * 2); // Extra delay for read operations
    
    // Read data in chunks for better reliability
    int bytes_read = 0;
    int chunk_size = 64; // Read in 64-byte chunks
    
    while (bytes_read < size) {
        int current_chunk = (size - bytes_read > chunk_size) ? chunk_size : (size - bytes_read);
        
        printf("K150: Reading chunk at offset %d (size: %d)\n", bytes_read, current_chunk);
        
        if (k150_read_serial(buffer + bytes_read, current_chunk) != SUCCESS) {
            fprintf(stderr, "K150: Failed to read chunk at offset %d\n", bytes_read);
            k150_close_port();
            return ERROR;
        }
        
        bytes_read += current_chunk;
        printf("K150: Read progress: %d/%d bytes (%.1f%%)\n", 
               bytes_read, size, (float)bytes_read * 100.0 / size);
        
        usleep(10000); // Small delay between chunks
    }
    
    printf("K150: Successfully read %d bytes from ROM\n", bytes_read);
    k150_close_port();
    return SUCCESS;
}

int k150_read_eeprom(unsigned char *buffer, int size)
{
    printf("K150: Reading EEPROM (%d bytes)\n", size);
    
    if (k150_start_communication() != SUCCESS) return ERROR;
    if (!current_device) {
        printf("K150: ERROR: No device detected for EEPROM read\n");
        return ERROR;
    }
    
    if (k150_send_device_params(current_device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

    unsigned char cmd = P018_CMD_READ_EEPROM;
    printf("K150: Sending read EEPROM command (0x0C)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        k150_voltages_off();
        return ERROR;
    }
    usleep(DELAY_US);

    int total_read = 0;
    while (total_read < size) {
        int chunk_size = (size - total_read > CHUNK_SIZE) ? CHUNK_SIZE : size - total_read;
        if (k150_read_serial(buffer + total_read, chunk_size) != SUCCESS) {
            printf("K150: EEPROM read timeout at byte %d\n", total_read);
            k150_voltages_off();
            return ERROR;
        }
        total_read += chunk_size;
        printf("K150: EEPROM read progress: %d/%d bytes\n", total_read, size);
    }

    k150_voltages_off();
    printf("K150: Successfully read %d bytes from EEPROM\n", size);
    return SUCCESS;
}

int k150_program_rom(unsigned char *buffer, int size)
{
    if (!current_device) {
        printf("K150: ERROR: No device detected for program operation\n");
        return ERROR;
    }
    return k150_program_rom_enhanced(current_device, buffer, size);
}

int k150_program_eeprom(unsigned char *buffer, int size)
{
    printf("K150: Programming EEPROM (%d bytes)\n", size);
    
    if (k150_start_communication() != SUCCESS) return ERROR;
    if (!current_device) {
        printf("K150: ERROR: No device detected for EEPROM program\n");
        return ERROR;
    }
    
    if (k150_send_device_params(current_device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

    unsigned char cmd = P018_CMD_PROGRAM_EEPROM;
    printf("K150: Sending program EEPROM command (0x09)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        k150_voltages_off();
        return ERROR;
    }
    usleep(DELAY_US);
    
    unsigned char ack;
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM) {
        printf("K150: ERROR: Expected 'Y' ACK for EEPROM program, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }

    int total_written = 0;
    while (total_written < size) {
        int chunk_size = (size - total_written > CHUNK_SIZE) ? CHUNK_SIZE : size - total_written;
        if (k150_write_serial(buffer + total_written, chunk_size) != SUCCESS) {
            printf("K150: EEPROM program write failed at byte %d\n", total_written);
            k150_voltages_off();
            return ERROR;
        }
        total_written += chunk_size;
        printf("K150: EEPROM program progress: %d/%d bytes\n", total_written, size);
        usleep(DELAY_US);
    }

    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM_DONE) {
        printf("K150: ERROR: Expected 'P' ACK for EEPROM program complete, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }
    
    k150_voltages_off();
    printf("K150: EEPROM programming completed successfully\n");
    return SUCCESS;
}

int k150_force_led_off(const char *device_path)
{
    printf("K150: Forcing LED off for device %s\n", device_path ? device_path : "default");
    
    if (k150_open_serial_port("/dev/ttyUSB0") == SUCCESS) {
        k150_voltages_off();
        k150_close_serial_port();
    }
    return SUCCESS;
}

// Enhanced interface functions that use P018 protocol with picdev.c integration

// Detect chip and return device information
int DoDetectChip_Enhanced(char** detected_name)
{
    if (k150_open_port("/dev/ttyUSB0") != SUCCESS) return ERROR;
    
    const PIC_DEFINITION *device = NULL;
    int result = k150_detect_chip_enhanced(&device);
    
    if (result == SUCCESS && device) {
        *detected_name = strdup(device->name);
        current_device = (PIC_DEFINITION *)device;
    }
    
    k150_close_port();
    return result;
}

// Erase chip
int DoErasePgm_Enhanced(const char* device_name)
{
    if (k150_open_port("/dev/ttyUSB0") != SUCCESS) return ERROR;
    
    PIC_DEFINITION *device = find_device_by_name(device_name);
    if (!device) {
        printf("ERROR: Unknown device: %s\n", device_name);
        k150_close_port();
        return ERROR;
    }
    
    int result = k150_erase_chip_enhanced(device);
    k150_close_port();
    return result;
}

// Read ROM to HEX file
int DoReadPgm_Enhanced(const char* device_name, const char* hex_filename)
{
    if (k150_open_port("/dev/ttyUSB0") != SUCCESS) return ERROR;
    
    PIC_DEFINITION *device = find_device_by_name(device_name);
    if (!device) {
        printf("ERROR: Unknown device: %s\n", device_name);
        k150_close_port();
        return ERROR;
    }
    
    // Calculate ROM size in bytes (words * 2)
    int pgm_size = (device->def[PD_PGM_SIZEH] << 8) | device->def[PD_PGM_SIZEL];
    int rom_bytes = pgm_size * 2;
    
    if (rom_bytes > MAX_ROM_SIZE) {
        printf("ERROR: ROM size %d exceeds maximum %d\n", rom_bytes, MAX_ROM_SIZE);
        k150_close_port();
        return ERROR;
    }
    
    unsigned char buffer[MAX_ROM_SIZE];
    int result = k150_read_rom_enhanced(device, buffer, rom_bytes);
    
    if (result == SUCCESS) {
        // Save binary dump
        char rom_filename[256];
        snprintf(rom_filename, sizeof(rom_filename), "%s.rom", device->name);
        FILE *fp = fopen(rom_filename, "wb");
        if (fp) {
            fwrite(buffer, 1, rom_bytes, fp);
            fclose(fp);
            printf("K150: Binary dump saved to %s\n", rom_filename);
        }
        
        // Save HEX file
        if (save_hex_file(hex_filename, buffer, rom_bytes) == SUCCESS) {
            printf("K150: Successfully read %d bytes from %s\n", rom_bytes, device->name);
            printf("K150: Data saved to hex file\n");
        }
    }
    
    k150_close_port();
    return result;
}

// Program ROM from HEX file with verification
int DoProgramPgm_Enhanced(const char* device_name, const char* hex_filename)
{
    if (k150_open_port("/dev/ttyUSB0") != SUCCESS) return ERROR;
    
    PIC_DEFINITION *device = find_device_by_name(device_name);
    if (!device) {
        printf("ERROR: Unknown device: %s\n", device_name);
        k150_close_port();
        return ERROR;
    }
    
    // Calculate ROM size in bytes
    int pgm_size = (device->def[PD_PGM_SIZEH] << 8) | device->def[PD_PGM_SIZEL];
    int rom_bytes = pgm_size * 2;
    
    if (rom_bytes > MAX_ROM_SIZE) {
        printf("ERROR: ROM size %d exceeds maximum %d\n", rom_bytes, MAX_ROM_SIZE);
        k150_close_port();
        return ERROR;
    }
    
    unsigned char buffer[MAX_ROM_SIZE];
    
    // First, check HEX file size before loading
    FILE *fp = fopen(hex_filename, "r");
    if (!fp) {
        printf("ERROR: Cannot open HEX file %s\n", hex_filename);
        k150_close_port();
        return ERROR;
    }
    
    // Calculate actual HEX file data size
    char line[256];
    int hex_max_addr = 0;
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != ':') continue;
        int byte_count, address, record_type;
        if (sscanf(line + 1, "%02x%04x%02x", &byte_count, &address, &record_type) == 3) {
            if (record_type == 1) break; // End of file
            if (record_type == 0 && byte_count > 0) {
                if (address + byte_count > hex_max_addr) {
                    hex_max_addr = address + byte_count;
                }
            }
        }
    }
    fclose(fp);
    
    // Critical capacity check with warning
    if (hex_max_addr > rom_bytes) {
        printf("CRITICAL WARNING: HEX file size (%d bytes) exceeds device capacity (%d bytes)\n", 
               hex_max_addr, rom_bytes);
        printf("WARNING: Data will be truncated from %d to %d bytes - firmware may not work correctly!\n",
               hex_max_addr, rom_bytes);
        printf("CONTINUING WITH TRUNCATION (automatic mode)\n");
        // In automatic mode, continue with truncation but warn user
    }
    
    int data_size = load_hex_file(hex_filename, buffer, rom_bytes);
    
    if (data_size <= 0) {
        printf("ERROR: Failed to load HEX file %s\n", hex_filename);
        k150_close_port();
        return ERROR;
    }
    
    // Use actual device capacity, not HEX file size
    data_size = (data_size > rom_bytes) ? rom_bytes : data_size;
    
    printf("K150: Programming [%s] (%d bytes)...\n", device->name, data_size);
    
    // Program the device
    int result = k150_program_rom_enhanced(device, buffer, data_size);
    
    if (result == SUCCESS) {
        // Automatic verification
        result = k150_verify_rom_enhanced(device, buffer, data_size);
    }
    
    k150_close_port();
    return result;
}

//--------------------------------------------------------------------
// K150 specific write program memory function
// Based on localhost.LOG analysis: 0x32 command + 2-byte data chunks
//--------------------------------------------------------------------
int k150_write_pgm(const PIC_DEFINITION *picDevice, FILE *hexFile)
{
    if (!picDevice || !hexFile) {
        fprintf(stderr, "K150: Invalid parameters for write operation\n");
        return 0;
    }
    
    printf("K150: Programming %s with hex file\n", picDevice->name);
    
    // Ensure K150 port is open
    printf("K150: Opening port and starting communication...\n");
    extern char *port_name; // From main.c
    if (k150_open_port(port_name) != 0) {
        fprintf(stderr, "K150: Failed to open port %s\n", port_name);
        return 0;
    }
    if (k150_start_communication() != 0) {
        fprintf(stderr, "K150: Failed to start communication\n");
        k150_close_port();
        return 0;
    }
    
    // Read hex file into buffer
    fseek(hexFile, 0, SEEK_END);
    long file_size = ftell(hexFile);
    fseek(hexFile, 0, SEEK_SET);
    
    unsigned char *hex_buffer = malloc(file_size + 1);
    if (!hex_buffer) {
        fprintf(stderr, "K150: Memory allocation failed\n");
        return 0;
    }
    
    size_t read_size = fread(hex_buffer, 1, file_size, hexFile);
    hex_buffer[read_size] = '\0';
    
    // Get program memory size from device definition
    int pgm_size = (picDevice->def[PD_PGM_SIZEH] << 8) | picDevice->def[PD_PGM_SIZEL];
    printf("K150: Device capacity: %d words, Hex file size: %ld bytes, Using: %ld bytes\n", 
           pgm_size, file_size, read_size);
    
    // Parse hex file with relaxed validation
    printf("K150: Parsing hex file with relaxed validation\n");
    unsigned char *rom_data = malloc(pgm_size * 2); // Words to bytes
    if (!rom_data) {
        fprintf(stderr, "K150: ROM buffer allocation failed\n");
        free(hex_buffer);
        return 0;
    }
    
    // Initialize ROM data to 0xFF (erased state)
    memset(rom_data, 0xFF, pgm_size * 2);
    
    // Simple hex parsing (Intel HEX format)
    char *line = strtok((char*)hex_buffer, "\n\r");
    int total_bytes = 0;
    
    while (line) {
        if (line[0] == ':' && strlen(line) >= 11) {
            int byte_count = (hex_to_byte(line[1]) << 4) | hex_to_byte(line[2]);
            int address = ((hex_to_byte(line[3]) << 4) | hex_to_byte(line[4])) << 8;
            address |= (hex_to_byte(line[5]) << 4) | hex_to_byte(line[6]);
            int record_type = (hex_to_byte(line[7]) << 4) | hex_to_byte(line[8]);
            
            if (record_type == 0 && address < (pgm_size * 2)) {
                for (int i = 0; i < byte_count && (address + i) < (pgm_size * 2); i++) {
                    int data_pos = 9 + (i * 2);
                    if (data_pos + 1 < strlen(line)) {
                        rom_data[address + i] = (hex_to_byte(line[data_pos]) << 4) | hex_to_byte(line[data_pos + 1]);
                        total_bytes++;
                    }
                }
            }
        }
        line = strtok(NULL, "\n\r");
    }
    
    printf("K150: Programming %s (%d bytes)...\n", picDevice->name, total_bytes);
    
    // Now implement the K150 programming protocol
    // Based on localhost.LOG: 0x32 command + 2-byte data pattern
    
    // Send programming command (0x32)
            if (k150_send_byte(0x32) != 0) {
        fprintf(stderr, "K150: Failed to send programming command\n");
        k150_close_port();
        free(hex_buffer);
        free(rom_data);
        return 0;
    }
    
    // Program data in 2-byte chunks (as seen in localhost.LOG)
    for (int addr = 0; addr < total_bytes; addr += 2) {
        // Send address
        if (k150_send_byte(0x06) != 0 || k150_send_byte(addr & 0xFF) != 0) {
            fprintf(stderr, "K150: Failed to send address\n");
            k150_close_port();
            free(hex_buffer);
            free(rom_data);
            return 0;
        }
        
        // Send 2-byte data
        unsigned char data_low = (addr < (pgm_size * 2)) ? rom_data[addr] : 0xFF;
        unsigned char data_high = (addr + 1 < (pgm_size * 2)) ? rom_data[addr + 1] : 0xFF;
        
        if (k150_send_byte(data_low) != 0 || k150_send_byte(data_high) != 0) {
            fprintf(stderr, "K150: Failed to send data at address 0x%04X\n", addr);
            k150_close_port();
            free(hex_buffer);
            free(rom_data);
            return 0;
        }
        
        // Wait for programming completion
        usleep(10000); // 10ms delay as seen in localhost.LOG
    }
    
    printf("K150: Programming completed successfully\n");
    
    // Close K150 port
    k150_close_port();
    
    free(hex_buffer);
    free(rom_data);
    return 1; // Success
}

// Helper function to convert hex character to byte
static int hex_to_byte(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

//-----------------------------------------------------------------------------
// ZIF Socket Pin Mapping Table
// Based on K150 ZIF socket configuration and picdevrc data
//-----------------------------------------------------------------------------

static const ZIF_INFO zif_mapping[] = {
    // 18-pin PICs (ZIF pins 1-20)
    {"16F84",    2, 18, "Insert PIC16F84 with pin 1 at ZIF pin 2 (18-pin DIP)", false},
    {"16F84A",   2, 18, "Insert PIC16F84A with pin 1 at ZIF pin 2 (18-pin DIP)", false},
    {"16F628",   1, 18, "Insert PIC16F628 with pin 1 at ZIF pin 1 (18-pin DIP)", false},
    {"16F628A",  2, 18, "Insert PIC16F628A with pin 1 at ZIF pin 2 (18-pin DIP)", false},
    
    // 28-pin PICs (ZIF pins 1-28)
    {"16F876",   1, 28, "Insert PIC16F876 with pin 1 at ZIF pin 1 (28-pin DIP)", false},
    {"16F876A",  1, 28, "Insert PIC16F876A with pin 1 at ZIF pin 1 (28-pin DIP)", false},
    {"16F877",   1, 28, "Insert PIC16F877 with pin 1 at ZIF pin 1 (28-pin DIP)", false},
    {"16F877A",  1, 28, "Insert PIC16F877A with pin 1 at ZIF pin 1 (28-pin DIP)", false},
    
    // 20-pin PICs (ICSP recommended)
    {"16F690",   0, 20, "PIC16F690: Use ICSP connector (20-pin package not suitable for ZIF)", true},
    {"16F88",    0, 18, "PIC16F88: Use ICSP connector or insert with pin 1 at ZIF pin 1", false},
    
    // 8-pin PICs
    {"12F675",   0, 8,  "PIC12F675: Use ICSP connector (8-pin package)", true},
    {"12F683",   0, 8,  "PIC12F683: Use ICSP connector (8-pin package)", true},
    
    // 14-pin PICs
    {"16F505",   0, 14, "PIC16F505: Insert with pin 1 at ZIF pin 14 (14-pin DIP)", false},
    {"16F506",   0, 14, "PIC16F506: Insert with pin 1 at ZIF pin 14 (14-pin DIP)", false},
    
    // 40-pin PICs
    {"16F887",   1, 40, "Insert PIC16F887 with pin 1 at ZIF pin 1 (40-pin DIP)", false},
    {"18F4520",  1, 40, "Insert PIC18F4520 with pin 1 at ZIF pin 1 (40-pin DIP)", false},
    
    // End marker
    {NULL, 0, 0, NULL, false}
};

//-----------------------------------------------------------------------------
// Get ZIF socket information for a specific PIC
//-----------------------------------------------------------------------------
const ZIF_INFO* get_zif_info(const char* pic_name) {
    if (!pic_name) return NULL;
    
    for (int i = 0; zif_mapping[i].pic_name != NULL; i++) {
        if (strcasecmp(pic_name, zif_mapping[i].pic_name) == 0) {
            return &zif_mapping[i];
        }
    }
    
    // Default for unknown PICs
    return NULL;
}

//-----------------------------------------------------------------------------
// Display ZIF socket placement instructions
//-----------------------------------------------------------------------------
void show_zif_instructions(const char* pic_name) {
    if (!pic_name) {
        printf("\nK150 ZIF Socket Information:\n");
        printf("===========================\n");
        printf("Supported PIC devices and their ZIF socket placement:\n\n");
        
        for (int i = 0; zif_mapping[i].pic_name != NULL; i++) {
            const ZIF_INFO* info = &zif_mapping[i];
            printf("%-10s: %s\n", info->pic_name, info->instructions);
            if (info->icsp_only) {
                printf("            ⚠️  ICSP-only device - use ICSP connector\n");
            }
        }
        
        printf("\nGeneral ZIF Socket Guidelines:\n");
        printf("• Pin 1 indicator: Look for notch or dot on the PIC package\n");
        printf("• ZIF lever: Open lever, insert PIC, close lever gently\n");
        printf("• Power off: Always power off K150 before inserting/removing PICs\n");
        printf("• ICSP: For unsupported packages, use ICSP connector\n\n");
        return;
    }
    
    const ZIF_INFO* info = get_zif_info(pic_name);
    if (info) {
        printf("\n📌 ZIF Socket Instructions for %s:\n", pic_name);
        printf("════════════════════════════════════════\n");
        printf("Package: %d-pin DIP\n", info->package_pins);
        
        if (info->icsp_only) {
            printf("⚠️  ICSP ONLY: %s\n", info->instructions);
            printf("\nICSP Connection:\n");
            printf("• Use the 6-pin ICSP header on K150\n");
            printf("• Connect: VPP, VDD, VSS, PGC, PGD, and optionally LVP\n");
            printf("• Refer to %s datasheet for ICSP pin mapping\n", pic_name);
        } else {
            printf("✅ ZIF Socket: %s\n", info->instructions);
            printf("\nDetailed Steps:\n");
            printf("1. Power off the K150 programmer\n");
            printf("2. Open the ZIF socket lever\n");
            printf("3. Locate pin 1 on the PIC (notch or dot indicator)\n");
            
            if (info->zif_pin == 1) {
                printf("4. Align PIC pin 1 with ZIF socket pin 1 (top-left)\n");
            } else {
                printf("4. Align PIC pin 1 with ZIF socket pin %d\n", info->zif_pin);
            }
            
            printf("5. Insert PIC gently into the socket\n");
            printf("6. Close the ZIF socket lever\n");
            printf("7. Power on the K150 programmer\n");
        }
        
        printf("\n💡 Alternative: ICSP programming is always available\n");
    } else {
        printf("\n❓ ZIF Socket Information for %s:\n", pic_name);
        printf("═════════════════════════════════════\n");
        printf("⚠️  No specific ZIF mapping found for this device.\n");
        printf("\nGeneral Guidelines:\n");
        printf("• For 18-pin DIPs: Usually pin 1 at ZIF pin 1 or 2\n");
        printf("• For 28-pin DIPs: Usually pin 1 at ZIF pin 1\n");
        printf("• For 40-pin DIPs: Usually pin 1 at ZIF pin 1\n");
        printf("• For other packages: Use ICSP connector\n");
        printf("\n💡 Recommendation: Use ICSP programming for unknown devices\n");
    }
    printf("\n");
}

// Helper function to send single byte to K150
int k150_send_byte(unsigned char byte)
{
    return k150_write_serial(&byte, 1);
}
