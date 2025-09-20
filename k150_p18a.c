// k150_p18a.c - Enhanced P18A Protocol Implementation
// Based on picpro Python implementation analysis

#define K150_P18A_MODULE_INCLUDED
#include "k150.h"
#include "picdev.h"
#include "k150_integration.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <time.h>
#include <stdlib.h>
#include <strings.h>

// Constants for compatibility
#ifndef SUCCESS
#define SUCCESS 0
#endif
#ifndef ERROR
#define ERROR 1
#endif

// Debug macro
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf("[P18A] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

// P18A Protocol Constants (from picpro analysis)
#define P18A_CMD_NOP            0x5A  // No operation
#define P18A_CMD_VERSION        0x5B  // Get version
#define P18A_CMD_RESET_POINTER  0x52  // Reset address pointer
#define P18A_CMD_WRITE_ROM      0x50  // Write to program memory
#define P18A_CMD_READ_ROM       0x51  // Read from program memory  
#define P18A_CMD_WRITE_EEPROM   0x53  // Write to EEPROM
#define P18A_CMD_READ_EEPROM    0x54  // Read from EEPROM
#define P18A_CMD_WRITE_CONFIG   0x55  // Write configuration
#define P18A_CMD_READ_CONFIG    0x56  // Read configuration
#define P18A_CMD_CHIP_ERASE     0x57  // Erase entire chip
#define P18A_CMD_WRITE_CAL      0x58  // Write calibration
#define P18A_CMD_READ_CAL       0x59  // Read calibration

// Response codes
#define P18A_RESP_OK            0x00  // Operation successful
#define P18A_RESP_ERROR         0xFF  // Operation failed
#define P18A_RESP_VERSION       0x03  // Version number

// Protocol timing (microseconds)
#define P18A_DELAY_CMD          1000   // 1ms between commands
#define P18A_DELAY_ERASE        20000  // 20ms for erase
#define P18A_DELAY_WRITE        5000   // 5ms for write
#define P18A_DELAY_READ         1000   // 1ms for read

// Buffer sizes
#define P18A_BLOCK_SIZE         32     // Block size for operations
#define P18A_MAX_RETRIES        3      // Maximum retry attempts

// Enhanced K150 state structure
typedef struct {
    int fd;                     // File descriptor
    int is_initialized;         // Initialization flag
    int firmware_version;       // Firmware version
    unsigned int address;       // Current address pointer
    const PIC_DEFINITION *device;  // Current device
} K150_STATE;

static K150_STATE k150_state = {-1, 0, 0, 0, NULL};

// Helper: Send command with optional data
static int p18a_send_command(unsigned char cmd, const unsigned char *data, int len)
{
    unsigned char buffer[256];
    int idx = 0;
    
    // Build command packet
    buffer[idx++] = cmd;
    
    // Add data if provided
    if (data && len > 0) {
        memcpy(&buffer[idx], data, len);
        idx += len;
    }
    
    // Send packet
    if (write(k150_state.fd, buffer, idx) != idx) {
        fprintf(stderr, "P18A: Failed to send command 0x%02X: %s\n", cmd, strerror(errno));
        return ERROR;
    }
    
    DEBUG_PRINT("P18A: Sent command 0x%02X with %d data bytes\n", cmd, len);
    usleep(P18A_DELAY_CMD);
    
    return SUCCESS;
}

// Helper: Read response
static int p18a_read_response(unsigned char *buffer, int expected_len)
{
    int total_read = 0;
    int retries = 0;
    
    while (total_read < expected_len && retries < P18A_MAX_RETRIES) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(k150_state.fd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms timeout
        
        if (select(k150_state.fd + 1, &readfds, NULL, NULL, &timeout) > 0) {
            int bytes = read(k150_state.fd, buffer + total_read, expected_len - total_read);
            if (bytes > 0) {
                total_read += bytes;
                DEBUG_PRINT("P18A: Read %d bytes (total: %d/%d)\n", bytes, total_read, expected_len);
            }
        } else {
            retries++;
            DEBUG_PRINT("P18A: Read timeout, retry %d/%d\n", retries, P18A_MAX_RETRIES);
        }
    }
    
    return (total_read == expected_len) ? SUCCESS : ERROR;
}

// Initialize P18A communication
int k150_p18a_init(const char *port_name)
{
    struct termios tty;
    
    // Open serial port
    k150_state.fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (k150_state.fd < 0) {
        fprintf(stderr, "P18A: Cannot open %s: %s\n", port_name, strerror(errno));
        return ERROR;
    }
    
    // Configure serial port (19200 8N1)
    memset(&tty, 0, sizeof(tty));
    tcgetattr(k150_state.fd, &tty);
    
    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 second timeout
    
    tcsetattr(k150_state.fd, TCSANOW, &tty);
    tcflush(k150_state.fd, TCIOFLUSH);
    
    // DTR/RTS initialization sequence (from Micropro.LOG)
    int status;
    ioctl(k150_state.fd, TIOCMGET, &status);
    
    // Clear DTR and RTS
    status &= ~(TIOCM_DTR | TIOCM_RTS);
    ioctl(k150_state.fd, TIOCMSET, &status);
    usleep(50000);
    
    // Set DTR and RTS
    status |= (TIOCM_DTR | TIOCM_RTS);
    ioctl(k150_state.fd, TIOCMSET, &status);
    usleep(50000);
    
    // Clear DTR (reset pulse)
    status &= ~TIOCM_DTR;
    ioctl(k150_state.fd, TIOCMSET, &status);
    usleep(100000);
    
    printf("P18A: Serial port %s initialized at 19200 baud\n", port_name);
    
    // Get firmware version
    if (p18a_send_command(P18A_CMD_VERSION, NULL, 0) == SUCCESS) {
        unsigned char version;
        if (p18a_read_response(&version, 1) == SUCCESS) {
            k150_state.firmware_version = version;
            printf("P18A: Firmware version: %d.%d\n", version >> 4, version & 0x0F);
        }
    }
    
    // Send NOP to verify communication
    if (p18a_send_command(P18A_CMD_NOP, NULL, 0) == SUCCESS) {
        unsigned char resp;
        if (p18a_read_response(&resp, 1) == SUCCESS && resp == P18A_RESP_OK) {
            printf("P18A: Communication established\n");
            k150_state.is_initialized = 1;
            return SUCCESS;
        }
    }
    
    fprintf(stderr, "P18A: Failed to establish communication\n");
    close(k150_state.fd);
    k150_state.fd = -1;
    return ERROR;
}

// Close P18A communication
void k150_p18a_close(void)
{
    if (k150_state.fd >= 0) {
        // Send NOP before closing
        p18a_send_command(P18A_CMD_NOP, NULL, 0);
        
        // Clear DTR/RTS
        int status;
        ioctl(k150_state.fd, TIOCMGET, &status);
        status &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(k150_state.fd, TIOCMSET, &status);
        
        close(k150_state.fd);
        k150_state.fd = -1;
        k150_state.is_initialized = 0;
        printf("P18A: Connection closed\n");
    }
}

// Reset address pointer
int k150_p18a_reset_pointer(void)
{
    if (!k150_state.is_initialized) return ERROR;
    
    if (p18a_send_command(P18A_CMD_RESET_POINTER, NULL, 0) == SUCCESS) {
        unsigned char resp;
        if (p18a_read_response(&resp, 1) == SUCCESS && resp == P18A_RESP_OK) {
            k150_state.address = 0;
            DEBUG_PRINT("P18A: Address pointer reset to 0x0000\n");
            return SUCCESS;
        }
    }
    return ERROR;
}

// Erase chip
int k150_p18a_erase_chip(void)
{
    if (!k150_state.is_initialized) return ERROR;
    
    printf("P18A: Erasing chip...\n");
    
    if (p18a_send_command(P18A_CMD_CHIP_ERASE, NULL, 0) == SUCCESS) {
        usleep(P18A_DELAY_ERASE);
        
        unsigned char resp;
        if (p18a_read_response(&resp, 1) == SUCCESS && resp == P18A_RESP_OK) {
            printf("P18A: Chip erased successfully\n");
            return SUCCESS;
        }
    }
    
    fprintf(stderr, "P18A: Chip erase failed\n");
    return ERROR;
}

// Read program memory
int k150_p18a_read_rom(unsigned char *buffer, int address, int length)
{
    if (!k150_state.is_initialized || !buffer) return ERROR;
    
    printf("P18A: Reading ROM from 0x%04X, length: %d bytes\n", address, length);
    
    // Reset pointer if needed
    if (k150_state.address != address) {
        if (k150_p18a_reset_pointer() != SUCCESS) return ERROR;
        k150_state.address = 0;
    }
    
    int total_read = 0;
    while (total_read < length) {
        int block_size = (length - total_read > P18A_BLOCK_SIZE) ? 
                         P18A_BLOCK_SIZE : (length - total_read);
        
        // Send read command with address and length
        unsigned char cmd_data[3];
        cmd_data[0] = (address + total_read) & 0xFF;
        cmd_data[1] = ((address + total_read) >> 8) & 0xFF;
        cmd_data[2] = block_size;
        
        if (p18a_send_command(P18A_CMD_READ_ROM, cmd_data, 3) != SUCCESS) {
            fprintf(stderr, "P18A: Failed to send read command at offset %d\n", total_read);
            return ERROR;
        }
        
        usleep(P18A_DELAY_READ);
        
        // Read response
        if (p18a_read_response(buffer + total_read, block_size) != SUCCESS) {
            fprintf(stderr, "P18A: Failed to read data at offset %d\n", total_read);
            return ERROR;
        }
        
        total_read += block_size;
        k150_state.address += block_size;
        
        printf("P18A: Read progress: %d/%d bytes (%.1f%%)\n", 
               total_read, length, (float)total_read * 100.0 / length);
    }
    
    printf("P18A: ROM read completed\n");
    return SUCCESS;
}

// Write program memory
int k150_p18a_write_rom(const unsigned char *buffer, int address, int length)
{
    if (!k150_state.is_initialized || !buffer) return ERROR;
    
    printf("P18A: Writing ROM to 0x%04X, length: %d bytes\n", address, length);
    
    // Reset pointer
    if (k150_p18a_reset_pointer() != SUCCESS) return ERROR;
    
    int total_written = 0;
    while (total_written < length) {
        int block_size = (length - total_written > P18A_BLOCK_SIZE) ? 
                         P18A_BLOCK_SIZE : (length - total_written);
        
        // Prepare write command
        unsigned char cmd_data[P18A_BLOCK_SIZE + 3];
        cmd_data[0] = (address + total_written) & 0xFF;
        cmd_data[1] = ((address + total_written) >> 8) & 0xFF;
        cmd_data[2] = block_size;
        memcpy(&cmd_data[3], buffer + total_written, block_size);
        
        if (p18a_send_command(P18A_CMD_WRITE_ROM, cmd_data, block_size + 3) != SUCCESS) {
            fprintf(stderr, "P18A: Failed to send write command at offset %d\n", total_written);
            return ERROR;
        }
        
        usleep(P18A_DELAY_WRITE);
        
        // Check response
        unsigned char resp;
        if (p18a_read_response(&resp, 1) != SUCCESS || resp != P18A_RESP_OK) {
            fprintf(stderr, "P18A: Write failed at offset %d (response: 0x%02X)\n", 
                    total_written, resp);
            return ERROR;
        }
        
        total_written += block_size;
        
        printf("P18A: Write progress: %d/%d bytes (%.1f%%)\n", 
               total_written, length, (float)total_written * 100.0 / length);
    }
    
    printf("P18A: ROM write completed\n");
    return SUCCESS;
}

// Read configuration word
int k150_p18a_read_config(unsigned int *config_word)
{
    if (!k150_state.is_initialized || !config_word) return ERROR;
    
    printf("P18A: Reading configuration word\n");
    
    if (p18a_send_command(P18A_CMD_READ_CONFIG, NULL, 0) != SUCCESS) {
        return ERROR;
    }
    
    usleep(P18A_DELAY_READ);
    
    unsigned char config_data[2];
    if (p18a_read_response(config_data, 2) != SUCCESS) {
        fprintf(stderr, "P18A: Failed to read configuration\n");
        return ERROR;
    }
    
    *config_word = (config_data[1] << 8) | config_data[0];
    printf("P18A: Configuration word: 0x%04X\n", *config_word);
    
    return SUCCESS;
}

// Write configuration word
int k150_p18a_write_config(unsigned int config_word)
{
    if (!k150_state.is_initialized) return ERROR;
    
    printf("P18A: Writing configuration word: 0x%04X\n", config_word);
    
    unsigned char config_data[2];
    config_data[0] = config_word & 0xFF;
    config_data[1] = (config_word >> 8) & 0xFF;
    
    if (p18a_send_command(P18A_CMD_WRITE_CONFIG, config_data, 2) != SUCCESS) {
        return ERROR;
    }
    
    usleep(P18A_DELAY_WRITE);
    
    unsigned char resp;
    if (p18a_read_response(&resp, 1) != SUCCESS || resp != P18A_RESP_OK) {
        fprintf(stderr, "P18A: Configuration write failed (response: 0x%02X)\n", resp);
        return ERROR;
    }
    
    printf("P18A: Configuration written successfully\n");
    return SUCCESS;
}

// Read EEPROM data
int k150_p18a_read_eeprom(unsigned char *buffer, int address, int length)
{
    if (!k150_state.is_initialized || !buffer) return ERROR;
    
    printf("P18A: Reading EEPROM from 0x%02X, length: %d bytes\n", address, length);
    
    int total_read = 0;
    while (total_read < length) {
        int block_size = (length - total_read > P18A_BLOCK_SIZE) ? 
                         P18A_BLOCK_SIZE : (length - total_read);
        
        unsigned char cmd_data[2];
        cmd_data[0] = address + total_read;
        cmd_data[1] = block_size;
        
        if (p18a_send_command(P18A_CMD_READ_EEPROM, cmd_data, 2) != SUCCESS) {
            return ERROR;
        }
        
        usleep(P18A_DELAY_READ);
        
        if (p18a_read_response(buffer + total_read, block_size) != SUCCESS) {
            fprintf(stderr, "P18A: Failed to read EEPROM at offset %d\n", total_read);
            return ERROR;
        }
        
        total_read += block_size;
        
        printf("P18A: EEPROM read progress: %d/%d bytes\n", total_read, length);
    }
    
    printf("P18A: EEPROM read completed\n");
    return SUCCESS;
}

// Write EEPROM data
int k150_p18a_write_eeprom(const unsigned char *buffer, int address, int length)
{
    if (!k150_state.is_initialized || !buffer) return ERROR;
    
    printf("P18A: Writing EEPROM to 0x%02X, length: %d bytes\n", address, length);
    
    int total_written = 0;
    while (total_written < length) {
        int block_size = (length - total_written > P18A_BLOCK_SIZE) ? 
                         P18A_BLOCK_SIZE : (length - total_written);
        
        unsigned char cmd_data[P18A_BLOCK_SIZE + 2];
        cmd_data[0] = address + total_written;
        cmd_data[1] = block_size;
        memcpy(&cmd_data[2], buffer + total_written, block_size);
        
        if (p18a_send_command(P18A_CMD_WRITE_EEPROM, cmd_data, block_size + 2) != SUCCESS) {
            return ERROR;
        }
        
        usleep(P18A_DELAY_WRITE);
        
        unsigned char resp;
        if (p18a_read_response(&resp, 1) != SUCCESS || resp != P18A_RESP_OK) {
            fprintf(stderr, "P18A: EEPROM write failed at offset %d\n", total_written);
            return ERROR;
        }
        
        total_written += block_size;
        
        printf("P18A: EEPROM write progress: %d/%d bytes\n", total_written, length);
    }
    
    printf("P18A: EEPROM write completed\n");
    return SUCCESS;
}

// Verify ROM data
int k150_p18a_verify_rom(const unsigned char *expected, int address, int length)
{
    unsigned char *read_buffer = malloc(length);
    if (!read_buffer) return ERROR;
    
    printf("P18A: Verifying ROM...\n");
    
    if (k150_p18a_read_rom(read_buffer, address, length) != SUCCESS) {
        free(read_buffer);
        return ERROR;
    }
    
    int errors = 0;
    for (int i = 0; i < length; i++) {
        if (read_buffer[i] != expected[i]) {
            if (errors < 10) {  // Report first 10 errors
                printf("P18A: Verify error at 0x%04X: expected 0x%02X, got 0x%02X\n",
                       address + i, expected[i], read_buffer[i]);
            }
            errors++;
        }
    }
    
    free(read_buffer);
    
    if (errors > 0) {
        printf("P18A: Verification failed with %d errors\n", errors);
        return ERROR;
    }
    
    printf("P18A: Verification successful\n");
    return SUCCESS;
}

// High-level program function
int k150_p18a_program_device(const char *device_name, const unsigned char *rom_data, 
                             int rom_size, unsigned int config_word)
{
    printf("P18A: Programming %s\n", device_name);
    
    // Find device in picdev array
    const PIC_DEFINITION *device = NULL;
    extern const PIC_DEFINITION *deviceArray[];
    
    for (int i = 0; deviceArray[i] != NULL; i++) {
        if (strcasecmp(deviceArray[i]->name, device_name) == 0) {
            device = deviceArray[i];
            break;
        }
    }
    
    if (!device) {
        fprintf(stderr, "P18A: Unknown device: %s\n", device_name);
        return ERROR;
    }
    
    k150_state.device = device;
    
    // Step 1: Erase chip
    if (k150_p18a_erase_chip() != SUCCESS) {
        fprintf(stderr, "P18A: Failed to erase chip\n");
        return ERROR;
    }
    
    // Step 2: Write ROM
    if (k150_p18a_write_rom(rom_data, 0, rom_size) != SUCCESS) {
        fprintf(stderr, "P18A: Failed to write ROM\n");
        return ERROR;
    }
    
    // Step 3: Write configuration
    if (k150_p18a_write_config(config_word) != SUCCESS) {
        fprintf(stderr, "P18A: Failed to write configuration\n");
        return ERROR;
    }
    
    // Step 4: Verify ROM
    if (k150_p18a_verify_rom(rom_data, 0, rom_size) != SUCCESS) {
        fprintf(stderr, "P18A: Verification failed\n");
        return ERROR;
    }
    
    printf("P18A: Device programmed successfully\n");
    return SUCCESS;
}

// Integration with existing k150.c functions
int k150_use_p18a_protocol(void)
{
    // Check if P18A protocol should be used based on firmware detection
    if (k150_state.firmware_version >= 0x30) {  // Version 3.0 or higher
        printf("P18A: Using enhanced P18A protocol (firmware %d.%d)\n",
               k150_state.firmware_version >> 4, k150_state.firmware_version & 0x0F);
        return 1;
    }
    return 0;
}

// Wrapper functions for compatibility
int k150_p18a_read_pgm(unsigned char *buffer, int size)
{
    return k150_p18a_read_rom(buffer, 0, size);
}

int k150_p18a_write_pgm(const unsigned char *buffer, int size)
{
    return k150_p18a_write_rom(buffer, 0, size);
}

int k150_p18a_erase(void)
{
    return k150_p18a_erase_chip();
}