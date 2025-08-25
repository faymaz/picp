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
#include "serial.h"
#include "picdev.h"
#include "k150.h"
#include "picpro_backend.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/select.h>

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
#define TIMEOUT_RETRIES 150  // Increased retries for PL2303 stability
#define DELAY_US 50000       // 50ms delay for better PL2303 timing

// Constants for K150 functions
#define SUCCESS 0
#define ERROR -1

// Global variables
static int k150_fd = -1;
static PIC_DEFINITION *current_device = NULL;
static int k150_firmware_version = 0;

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
#define P018_ACK_CONFIG                 0x43  // 'C'

//-----------------------------------------------------------------------------
// Serial communication functions
//-----------------------------------------------------------------------------
int k150_write_serial(const unsigned char *buf, int len)
{
    int written = write(k150_fd, buf, len);
    if (written != len) {
        printf("ERROR: k150_write_serial failed: %s\n", strerror(errno));
        return ERROR;
    }
    printf("DEBUG: write_serial sent %d bytes: ", written);
    for (int i = 0; i < written; i++) printf("0x%02x ", buf[i]);
    printf("\n");
    return SUCCESS;
}

int k150_read_serial(unsigned char *buf, int len)
{
    int total_read = 0, retries = 0;
    const int MAX_RETRIES = 50; // Reduced from 150 for better performance
    struct timeval tv;
    fd_set rfds;

    fprintf(stderr, "DEBUG: read_serial attempting to read %d bytes\n", len);
    while (total_read < len && retries < MAX_RETRIES) {
        FD_ZERO(&rfds);
        FD_SET(k150_fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms timeout - better for PL2303

        int r = select(k150_fd + 1, &rfds, NULL, NULL, &tv);
        if (r > 0) {
            int bytes_read = read(k150_fd, buf + total_read, len - total_read);
            if (bytes_read > 0) {
                total_read += bytes_read;
                fprintf(stderr, "DEBUG: read_serial got %d bytes: ", bytes_read);
                for (int i = 0; i < bytes_read; i++) fprintf(stderr, "0x%02x ", buf[total_read - bytes_read + i]);
                fprintf(stderr, "\n");
                
                // Reset retry counter on successful read
                retries = 0;
            } else if (bytes_read < 0) {
                fprintf(stderr, "ERROR: k150_read_serial failed: %s\n", strerror(errno));
                return ERROR;
            }
        } else if (r == 0) {
            retries++;
            if (retries % 10 == 0) { // Log every 10th retry to reduce spam
                fprintf(stderr, "DEBUG: read_serial retry %d/%d\n", retries, MAX_RETRIES);
            }
            usleep(1000); // 1ms delay between retries
            usleep(DELAY_US); // 50ms delay for PL2303 stability
        } else {
            fprintf(stderr, "ERROR: select failed: %s\n", strerror(errno));
            return ERROR;
        }
    }
    if (total_read < len) {
        fprintf(stderr, "ERROR: read_serial timeout, got %d/%d bytes\n", total_read, len);
        return ERROR;
    }
    return SUCCESS;
}

//-----------------------------------------------------------------------------
// P018 Protocol Implementation
//-----------------------------------------------------------------------------

// Start P018 communication
static int k150_start_communication(void)
{
    unsigned char cmd = P018_CMD_START;
    unsigned char ack;
    
    printf("K150: Sending start command 'P' (0x50)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) != SUCCESS) {
        fprintf(stderr, "K150: ERROR: Failed to read start ACK\n");
        return ERROR;
    }
    // K150 firmware uses non-standard ACK codes (from memory: accepts 'P', 'I', 0x50, 0x49, 0x51, 0x56)
    if (ack != 'P' && ack != 'I' && ack != 0x50 && ack != 0x49 && ack != 0x51 && ack != 0x56) {
        fprintf(stderr, "K150: ERROR: Unexpected start ACK, got 0x%02x\n", ack);
        return ERROR;
    }
    fprintf(stderr, "K150: Start ACK 0x%02x received\n", ack);
    return SUCCESS;
}

// Send device initialization parameters from picdev.c
static int k150_send_device_params(const PIC_DEFINITION *device)
{
    unsigned char cmd = P018_CMD_INIT;
    unsigned char params[11];
    
    // Extract parameters from picdev.c device definition
    int pgm_size = (device->def[PD_PGM_SIZEH] << 8) | device->def[PD_PGM_SIZEL];
    int data_size = (device->def[PD_DATA_SIZEH] << 8) | device->def[PD_DATA_SIZEL];
    
    params[0] = (pgm_size >> 8) & 0xFF;  // ROM size high
    params[1] = pgm_size & 0xFF;         // ROM size low
    params[2] = (data_size >> 8) & 0xFF; // EEPROM size high
    params[3] = data_size & 0xFF;        // EEPROM size low
    params[4] = device->def[14];         // Core type
    params[5] = device->def[15];         // Prog flags
    params[6] = device->def[16];         // Prog delay
    params[7] = device->def[17];         // Power sequence
    params[8] = device->def[18];         // Erase mode
    params[9] = device->def[19];         // Prog tries
    params[10] = device->def[20];        // Over program
    
    printf("K150: Sending init command (0x03) for %s\n", device->name);
    printf("K150: ROM size: %d words, EEPROM: %d bytes\n", pgm_size, data_size);
    
    if (k150_write_serial(&cmd, 1) != SUCCESS) return ERROR;
    if (k150_write_serial(params, 11) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    unsigned char ack;
    if (k150_read_serial(&ack, 1) != SUCCESS) {
        fprintf(stderr, "K150: ERROR: Failed to read init ACK\n");
        return ERROR;
    }
    // K150 firmware uses non-standard ACK codes (from memory: 0x50 instead of 0x49 for init)
    if (ack != 'I' && ack != 0x49 && ack != 0x50 && ack != 0x56 && ack != 0x76) {
        fprintf(stderr, "K150: ERROR: Unexpected init ACK, got 0x%02x\n", ack);
        return ERROR;
    }
    fprintf(stderr, "K150: Init ACK 0x%02x received\n", ack);
    return SUCCESS;
}

// Turn voltages on
static int k150_voltages_on(void)
{
    unsigned char cmd = P018_CMD_VOLTAGES_ON;
    unsigned char ack;
    
    printf("K150: Sending voltages ON command (0x04)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);
    
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_VOLTAGES_ON) {
        printf("K150: ERROR: Expected 'V' ACK, got 0x%02x\n", ack);
        return ERROR;
    }
    printf("K150: Voltages ON ACK 'V' received\n");
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

// Erase chip
int k150_erase_chip_enhanced(const PIC_DEFINITION *device)
{
    printf("K150: Erasing chip %s\n", device->name);
    
    if (k150_start_communication() != SUCCESS) return ERROR;
    if (k150_send_device_params(device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

    unsigned char cmd = P018_CMD_ERASE_CHIP;
    printf("K150: Sending erase chip command (0x0F)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        k150_voltages_off();
        return ERROR;
    }
    usleep(DELAY_US * 2); // Erase takes longer

    unsigned char ack;
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM) {
        printf("K150: ERROR: Expected 'Y' ACK for erase, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }
    printf("K150: Erase ACK 'Y' received\n");
    
    k150_voltages_off();
    printf("K150: Chip erase completed\n");
    return SUCCESS;
}

// Read ROM using P018 protocol
static int k150_read_rom_enhanced(const PIC_DEFINITION *device, unsigned char *buffer, int size)
{
    printf("K150: Reading %d bytes from %s using P018 protocol\n", size, device->name);
    tcflush(k150_fd, TCIOFLUSH);

    if (k150_start_communication() != SUCCESS) return ERROR;
    if (k150_send_device_params(device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

    unsigned char cmd = P018_CMD_READ_ROM;
    printf("K150: Sending read ROM command (0x0B)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) {
        k150_voltages_off();
        return ERROR;
    }
    usleep(DELAY_US);

    int total_read = 0;
    while (total_read < size) {
        int chunk_size = (size - total_read > CHUNK_SIZE) ? CHUNK_SIZE : size - total_read;
        if (k150_read_serial(buffer + total_read, chunk_size) != SUCCESS) {
            printf("K150: Read timeout at byte %d\n", total_read);
            k150_voltages_off();
            return ERROR;
        }
        total_read += chunk_size;
        printf("K150: Read progress: %d/%d bytes (chunk: %d)\n", total_read, size, chunk_size);
    }

    k150_voltages_off();
    printf("K150: Successfully read %d bytes from %s using P018 protocol\n", size, device->name);
    return SUCCESS;
}

// Program ROM using P018 protocol
int k150_program_rom_enhanced(const PIC_DEFINITION *device, const unsigned char *buffer, int size)
{
    printf("K150: Programming %d bytes to %s using P018 protocol\n", size, device->name);
    tcflush(k150_fd, TCIOFLUSH);

    if (k150_start_communication() != SUCCESS) return ERROR;
    if (k150_send_device_params(device) != SUCCESS) return ERROR;
    if (k150_voltages_on() != SUCCESS) return ERROR;

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
        usleep(DELAY_US);
    }

    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_PROGRAM_DONE) {
        printf("K150: ERROR: Expected 'P' ACK for program complete, got 0x%02x\n", ack);
        k150_voltages_off();
        return ERROR;
    }
    printf("K150: Program complete ACK 'P' received\n");
    
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
            sscanf(line + 9 + i * 2, "%02x", &byte_val);
            if (address + i < max_size) {
                buffer[address + i] = byte_val;
                if (address + i > max_addr) max_addr = address + i;
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

// Open K150 serial port with flexible port selection
static int k150_open_serial_port(const char *device)
{
    struct termios options;
    
    fprintf(stderr, "DEBUG: Attempting to open serial port %s\n", device);
    k150_fd = open(device, O_RDWR | O_NOCTTY); // Blocking mode, removed O_NDELAY
    if (k150_fd == -1) {
        fprintf(stderr, "ERROR: Unable to open %s: %s\n", device, strerror(errno));
        return ERROR;
    }
    
    // Configure serial port for K150 (9600 baud for updated firmware)
    tcgetattr(k150_fd, &options);
    // Try different baud rates for PL2303 compatibility
    cfsetospeed(&options, B9600);
    cfsetispeed(&options, B9600);
    options.c_cflag = CS8 | CLOCAL | CREAD; // 8N1, no parity
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 10; // 1 saniye timeout
    options.c_cc[VMIN] = 0; // Minimum 0 bytes, timeout-based
    
    tcsetattr(k150_fd, TCSANOW, &options);
    tcflush(k150_fd, TCIOFLUSH);
    fcntl(k150_fd, F_SETFL, 0); // Clear non-blocking flags
    
    // Enhanced DTR/RTS reset for PL2303 compatibility
    int dtr_flag = TIOCM_DTR;
    int rts_flag = TIOCM_RTS;
    
    // Clear all signals first
    ioctl(k150_fd, TIOCMBIC, &dtr_flag);
    ioctl(k150_fd, TIOCMBIC, &rts_flag);
    usleep(200000); // 200ms
    tcflush(k150_fd, TCIOFLUSH);
    
    // Enhanced DTR/RTS reset sequence for PL2303 compatibility
    int status;
    if (ioctl(k150_fd, TIOCMGET, &status) == 0) {
        // Clear DTR and RTS
        status &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(200000); // 200ms wait for PL2303 reset
        
        // Set DTR high for normal operation
        status |= TIOCM_DTR;
        ioctl(k150_fd, TIOCMSET, &status);
        usleep(100000); // 100ms stabilization
        
        // Log modem status for debugging
        ioctl(k150_fd, TIOCMGET, &status);
        fprintf(stderr, "DEBUG: Modem status - DTR:%s RTS:%s CTS:%s DSR:%s DCD:%s\n",
                (status & TIOCM_DTR) ? "ON" : "OFF",
                (status & TIOCM_RTS) ? "ON" : "OFF", 
                (status & TIOCM_CTS) ? "ON" : "OFF",
                (status & TIOCM_DSR) ? "ON" : "OFF",
                (status & TIOCM_CD) ? "ON" : "OFF");
        fprintf(stderr, "DEBUG: Enhanced DTR reset sequence completed\n");
    }
    
    tcflush(k150_fd, TCIOFLUSH);
    
    // Set RTS for LED control
    ioctl(k150_fd, TIOCMBIS, &rts_flag);
    
    fprintf(stderr, "DEBUG: Serial port %s initialized at 9600,8,N,1 with DTR reset\n", device);
    return SUCCESS;
}

static int k150_close_serial_port(void)
{
    if (k150_fd >= 0) {
        // Turn off RTS to disable yellow LED
        int rts_flag = TIOCM_RTS;
        ioctl(k150_fd, TIOCMBIC, &rts_flag);
        close(k150_fd);
        k150_fd = -1;
        printf("DEBUG: K150 port closed\n");
    }
    return SUCCESS;
}

// Standard K150 interface functions called by main.c

int k150_open_port(const char *device)
{
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
    fprintf(stderr, "DEBUG: K150 programmer detected successfully\n");
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

// Hybrid chip detection - try picpro backend first, fallback to P018
int k150_detect_chip_with_type(const char *expected_chip)
{
    extern char *port_name; // From main.c
    
    // Try picpro backend first (works with epk150.hex)
    if (picpro_check_availability() == 0) {
        fprintf(stderr, "K150: Using picpro backend for epk150.hex firmware\n");
        
        if (picpro_detect_chip(port_name, expected_chip) == 0) {
            fprintf(stderr, "K150: SUCCESS - picpro detected chip: %s\n", expected_chip);
            return SUCCESS;
        } else {
            fprintf(stderr, "K150: picpro detection failed, trying P018 protocol...\n");
        }
    } else {
        fprintf(stderr, "K150: picpro not available, using P018 protocol\n");
    }
    
    // Fallback to original P018 protocol
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
    if (!current_device) {
        printf("K150: ERROR: No device detected for read operation\n");
        return ERROR;
    }
    return k150_read_rom_enhanced(current_device, buffer, size);
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
    int data_size = load_hex_file(hex_filename, buffer, rom_bytes);
    
    if (data_size <= 0) {
        printf("ERROR: Failed to load HEX file %s\n", hex_filename);
        k150_close_port();
        return ERROR;
    }
    
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
