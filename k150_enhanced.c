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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>

// P018 Protocol Commands (based on softprotocol5.txt)
#define P018_CMD_START          'P'  // 0x50 - Start communication
#define P018_CMD_INIT           3    // Initialize with device parameters
#define P018_CMD_VOLTAGES_ON    4    // Turn voltages on
#define P018_CMD_VOLTAGES_OFF   5    // Turn voltages off
#define P018_CMD_PROGRAM_ROM    7    // Program ROM
#define P018_CMD_READ_ROM       11   // Read ROM
#define P018_CMD_READ_CONFIG    13   // Read configuration (chip detection)
#define P018_CMD_ERASE_CHIP     15   // Erase chip

// P018 Protocol ACK responses
#define P018_ACK_START          'P'  // 0x50 - Start ACK
#define P018_ACK_INIT           'I'  // 0x49 - Init ACK
#define P018_ACK_VOLTAGES_ON    'V'  // 0x56 - Voltages on ACK
#define P018_ACK_VOLTAGES_OFF   'v'  // 0x76 - Voltages off ACK
#define P018_ACK_PROGRAM        'Y'  // 0x59 - Program/Erase ACK
#define P018_ACK_PROGRAM_DONE   'P'  // 0x50 - Program complete ACK
#define P018_ACK_CONFIG         'C'  // 0x43 - Configuration start

#define MAX_ROM_SIZE 16384  // Maximum ROM size in bytes (8192 words)
#define CHUNK_SIZE 64       // Read/write chunk size
#define TIMEOUT_RETRIES 20  // Serial timeout retries
#define DELAY_US 5000       // 5ms delay between operations

// Constants for K150 functions
#define SUCCESS 0
#define ERROR -1

// Global variables
static int k150_fd = -1;  // File descriptor for K150 serial port
static PIC_DEFINITION *current_device = NULL;  // Currently selected device

// External device list from picdev.c
extern const PIC_DEFINITION *deviceArray[];

//-----------------------------------------------------------------------------
// Serial communication functions
//-----------------------------------------------------------------------------
static int k150_write_serial(const unsigned char *buf, int len)
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

static int k150_read_serial(unsigned char *buf, int len)
{
    int total_read = 0, retries = TIMEOUT_RETRIES;
    while (total_read < len && retries > 0) {
        int r = read(k150_fd, buf + total_read, len - total_read);
        if (r > 0) {
            total_read += r;
            printf("DEBUG: read_serial got %d bytes: ", r);
            for (int i = 0; i < r; i++) printf("0x%02x ", buf[i]);
            printf("\n");
        } else if (r == 0) {
            usleep(DELAY_US);
            retries--;
        } else {
            printf("ERROR: k150_read_serial failed: %s\n", strerror(errno));
            return ERROR;
        }
    }
    if (total_read < len) {
        printf("ERROR: k150_read_serial timeout, got %d/%d bytes\n", total_read, len);
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
    
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_START) {
        printf("K150: ERROR: Expected 'P' ACK, got 0x%02x\n", ack);
        return ERROR;
    }
    printf("K150: Start ACK 'P' received\n");
    return SUCCESS;
}

// Send device initialization parameters from picdev.c
static int k150_send_device_params(PIC_DEFINITION *device)
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
    if (k150_read_serial(&ack, 1) != SUCCESS || ack != P018_ACK_INIT) {
        printf("K150: ERROR: Expected 'I' ACK, got 0x%02x\n", ack);
        return ERROR;
    }
    printf("K150: Init ACK 'I' received\n");
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
static int k150_detect_chip_enhanced(PIC_DEFINITION **detected_device)
{
    if (k150_start_communication() != SUCCESS) return ERROR;
    
    unsigned char cmd = P018_CMD_READ_CONFIG;
    printf("K150: Sending read configuration command (0x0D)\n");
    if (k150_write_serial(&cmd, 1) != SUCCESS) return ERROR;
    usleep(DELAY_US);

    unsigned char config[30];
    if (k150_read_serial(config, 30) != SUCCESS) {
        printf("K150: Failed to read configuration\n");
        return ERROR;
    }

    if (config[0] != P018_ACK_CONFIG) {
        printf("K150: ERROR: Expected 'C' start for configuration, got 0x%02x\n", config[0]);
        return ERROR;
    }

    int chip_id = (config[2] << 8) | config[1]; // ChipID_H, ChipID_L
    printf("K150: Detected Chip ID: 0x%04x\n", chip_id);

    // Search deviceArray for matching chip ID
    for (int i = 0; deviceArray[i] != NULL; i++) {
        // Extract chip ID from device definition
        int dev_chip_id = (deviceArray[i]->def[22] << 8) | deviceArray[i]->def[23];
        if (dev_chip_id == chip_id) {
            printf("K150: Detected device: %s\n", deviceArray[i]->name);
            *detected_device = (PIC_DEFINITION *)deviceArray[i];
            return SUCCESS;
        }
    }
    printf("K150: Unknown chip ID 0x%04x\n", chip_id);
    return ERROR;
}

// Erase chip
static int k150_erase_chip_enhanced(PIC_DEFINITION *device)
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
static int k150_read_rom_enhanced(PIC_DEFINITION *device, unsigned char *buffer, int size)
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
static int k150_program_rom_enhanced(PIC_DEFINITION *device, const unsigned char *buffer, int size)
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
static int k150_verify_rom_enhanced(PIC_DEFINITION *device, const unsigned char *original_buffer, int size)
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

//-----------------------------------------------------------------------------
// Public interface functions
//-----------------------------------------------------------------------------

// Open K150 serial port
int k150_open_port(void)
{
    struct termios options;
    
    if (k150_fd != -1) {
        printf("K150: Port already open\n");
        return SUCCESS;
    }
    
    k150_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (k150_fd == -1) {
        printf("ERROR: Cannot open /dev/ttyUSB0: %s\n", strerror(errno));
        return ERROR;
    }
    
    // Configure serial port for P018 protocol (19200,N,8,1)
    tcgetattr(k150_fd, &options);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 1; // 100ms timeout
    options.c_cc[VMIN] = 0;
    
    tcsetattr(k150_fd, TCSANOW, &options);
    tcflush(k150_fd, TCIOFLUSH);
    
    printf("DEBUG: Serial port /dev/ttyUSB0 initialized at 19200,N,8,1\n");
    return SUCCESS;
}

// Close K150 serial port
int k150_close_port(void)
{
    if (k150_fd != -1) {
        // Turn off RTS to disable yellow LED
        int rts_flag = TIOCM_RTS;
        ioctl(k150_fd, TIOCMBIC, &rts_flag);
        close(k150_fd);
        k150_fd = -1;
        printf("DEBUG: K150 port closed\n");
    }
    return SUCCESS;
}

// Enhanced interface functions that use P018 protocol with picdev.c integration

// Detect chip and return device information
int DoDetectChip_Enhanced(char** detected_name)
{
    if (k150_open_port() != SUCCESS) return ERROR;
    
    PIC_DEFINITION *device = NULL;
    int result = k150_detect_chip_enhanced(&device);
    
    if (result == SUCCESS && device) {
        *detected_name = strdup(device->name);
        current_device = device;
    }
    
    k150_close_port();
    return result;
}

// Erase chip
int DoErasePgm_Enhanced(const char* device_name)
{
    if (k150_open_port() != SUCCESS) return ERROR;
    
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
    if (k150_open_port() != SUCCESS) return ERROR;
    
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
    if (k150_open_port() != SUCCESS) return ERROR;
    
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
