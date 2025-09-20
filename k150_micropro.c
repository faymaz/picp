// k150_micropro.c - Micropro.exe compatibility layer
// Based on Micropro.LOG and Micropro2.LOG analysis

#include "k150.h"
#include "k150_integration.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <errno.h>
#include <time.h>
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
#define DEBUG_PRINT(fmt, ...) printf("[MICROPRO] " fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

// Micropro.exe specific protocol sequences (from LOG analysis)
typedef struct {
    const char *name;
    unsigned char init_seq[16];
    int init_len;
    unsigned char erase_seq[8];
    int erase_len;
    unsigned char write_cmd;
    unsigned char read_cmd;
    int block_size;
    int write_delay_ms;
    int erase_delay_ms;
} MICROPRO_DEVICE;

// Device-specific sequences from Micropro.LOG analysis
static const MICROPRO_DEVICE micropro_devices[] = {
    {
        .name = "PIC16F84",
        .init_seq = {0x50, 0x03, 0x04, 0x00, 0x00, 0x40, 0x06, 0x00, 0xC8, 0x02, 0x00, 0x01, 0x00, 0x14},
        .init_len = 14,
        .erase_seq = {0x0F, 0x3F},
        .erase_len = 2,
        .write_cmd = 0x07,
        .read_cmd = 0x0B,
        .block_size = 16,
        .write_delay_ms = 10,
        .erase_delay_ms = 20
    },
    {
        .name = "PIC16F628A",
        .init_seq = {0x50, 0x03, 0x04, 0x00, 0x00, 0x80, 0x07, 0x00, 0xC8, 0x02, 0x00, 0x01, 0x00, 0x14},
        .init_len = 14,
        .erase_seq = {0x0F, 0x3F},
        .erase_len = 2,
        .write_cmd = 0x07,
        .read_cmd = 0x0B,
        .block_size = 32,
        .write_delay_ms = 5,
        .erase_delay_ms = 10
    },
    {
        .name = "PIC16F887",
        .init_seq = {0x50, 0x03, 0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0xC8, 0x02, 0x00, 0x01, 0x00, 0x14},
        .init_len = 14,
        .erase_seq = {0x18}, // Bulk erase command for 16F887
        .erase_len = 1,
        .write_cmd = 0x07,
        .read_cmd = 0x0B,
        .block_size = 64,
        .write_delay_ms = 3,
        .erase_delay_ms = 15
    }
};

// Micropro.LOG command sequence structure
typedef struct {
    unsigned char command;
    unsigned char params[8];
    int param_count;
    unsigned char expected_ack;
    int delay_ms;
    const char *description;
} MICROPRO_SEQUENCE;

// Standard Micropro.exe initialization sequence
static const MICROPRO_SEQUENCE micropro_init[] = {
    {0x00, {}, 0, 0x00, 10, "DTR/RTS clear"},           // Line 8-9
    {0x01, {}, 0, 0x00, 10, "DTR/RTS set"},            // Line 14-15  
    {0x02, {}, 0, 0x00, 10, "DTR clear"},              // Line 20
    {0x42, {0x03, 0x42}, 2, 0x42, 100, "Auto-response"}, // Line 26
    {0x50, {0x03}, 1, 0x50, 50, "Start command"},       // Line 32
    {0xFF, {}, 0, 0x00, 0, "End"}
};

// Global Micropro state
static struct {
    int fd;
    const MICROPRO_DEVICE *device;
    int is_connected;
    unsigned int current_address;
    unsigned char last_response;
} micropro_state = {-1, NULL, 0, 0, 0};

// Helper: Execute DTR/RTS control sequence
static int micropro_dtr_rts_control(int clear_dtr, int clear_rts)
{
    int status;
    if (ioctl(micropro_state.fd, TIOCMGET, &status) != 0) {
        return ERROR;
    }
    
    if (clear_dtr) status &= ~TIOCM_DTR;
    else status |= TIOCM_DTR;
    
    if (clear_rts) status &= ~TIOCM_RTS;
    else status |= TIOCM_RTS;
    
    if (ioctl(micropro_state.fd, TIOCMSET, &status) != 0) {
        return ERROR;
    }
    
    return SUCCESS;
}

// Helper: Wait for specific response
static int micropro_wait_response(unsigned char expected, int timeout_ms)
{
    unsigned char buffer[256];
    int total_wait = 0;
    
    while (total_wait < timeout_ms) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(micropro_state.fd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms
        
        if (select(micropro_state.fd + 1, &readfds, NULL, NULL, &timeout) > 0) {
            int bytes = read(micropro_state.fd, buffer, sizeof(buffer));
            if (bytes > 0) {
                for (int i = 0; i < bytes; i++) {
                    if (buffer[i] == expected) {
                        micropro_state.last_response = expected;
                        DEBUG_PRINT("Micropro: Got expected response 0x%02X\n", expected);
                        return SUCCESS;
                    }
                }
                // Log unexpected responses
                DEBUG_PRINT("Micropro: Unexpected response: ");
                for (int i = 0; i < bytes; i++) {
                    DEBUG_PRINT("0x%02X ", buffer[i]);
                }
                DEBUG_PRINT("\n");
            }
        }
        
        total_wait += 10;
        usleep(10000);
    }
    
    DEBUG_PRINT("Micropro: Timeout waiting for 0x%02X after %dms\n", expected, timeout_ms);
    return ERROR;
}

// Execute Micropro.exe compatible initialization
int micropro_init_sequence(void)
{
    printf("Micropro: Executing Micropro.exe initialization sequence\n");
    
    // Step 1: Clear DTR and RTS
    micropro_dtr_rts_control(1, 1);
    usleep(50000);
    
    // Step 2: Set DTR and RTS
    micropro_dtr_rts_control(0, 0);
    usleep(50000);
    
    // Step 3: Clear DTR only (reset pulse)
    micropro_dtr_rts_control(1, 0);
    usleep(100000);
    
    // Step 4: Check for auto-response (0x42 0x03 0x42)
    unsigned char auto_resp[3];
    int bytes = read(micropro_state.fd, auto_resp, 3);
    if (bytes == 3 && auto_resp[0] == 0x42 && auto_resp[1] == 0x03 && auto_resp[2] == 0x42) {
        printf("Micropro: Auto-response received (K150 detected)\n");
    } else {
        printf("Micropro: No auto-response (continuing anyway)\n");
    }
    
    // Step 5: Send start command (0x50 0x03)
    unsigned char start_cmd[] = {0x50, 0x03};
    if (write(micropro_state.fd, start_cmd, 2) != 2) {
        fprintf(stderr, "Micropro: Failed to send start command\n");
        return ERROR;
    }
    
    usleep(50000);
    
    // Check for ACK
    if (micropro_wait_response(0x50, 100) == SUCCESS) {
        printf("Micropro: Start command acknowledged\n");
        micropro_state.is_connected = 1;
        return SUCCESS;
    }
    
    return ERROR;
}

// Send device-specific initialization parameters
int micropro_send_device_params(const char *device_name)
{
    // Find device configuration
    micropro_state.device = NULL;
    for (int i = 0; i < sizeof(micropro_devices)/sizeof(micropro_devices[0]); i++) {
        if (strcasecmp(micropro_devices[i].name, device_name) == 0) {
            micropro_state.device = &micropro_devices[i];
            break;
        }
    }
    
    if (!micropro_state.device) {
        // Use default PIC16F84 parameters
        micropro_state.device = &micropro_devices[0];
        printf("Micropro: Using default parameters for %s\n", device_name);
    }
    
    printf("Micropro: Sending parameters for %s\n", micropro_state.device->name);
    
    // Send initialization sequence
    for (int i = 2; i < micropro_state.device->init_len; i++) {
        if (write(micropro_state.fd, &micropro_state.device->init_seq[i], 1) != 1) {
            fprintf(stderr, "Micropro: Failed to send parameter byte %d\n", i);
            return ERROR;
        }
        usleep(10000); // 10ms between bytes
    }
    
    printf("Micropro: Device parameters sent\n");
    return SUCCESS;
}

// Micropro.exe compatible erase
int micropro_erase_chip(void)
{
    if (!micropro_state.is_connected || !micropro_state.device) return ERROR;
    
    printf("Micropro: Erasing chip using sequence for %s\n", micropro_state.device->name);
    
    // Turn on programming voltage
    unsigned char voltage_on = 0x04;
    write(micropro_state.fd, &voltage_on, 1);
    usleep(50000);
    
    // Send erase sequence
    if (write(micropro_state.fd, micropro_state.device->erase_seq, 
              micropro_state.device->erase_len) != micropro_state.device->erase_len) {
        fprintf(stderr, "Micropro: Failed to send erase command\n");
        return ERROR;
    }
    
    // Wait for erase completion
    usleep(micropro_state.device->erase_delay_ms * 1000);
    
    // Check for completion
    if (micropro_wait_response(0x59, 1000) == SUCCESS) {  // 'Y' = success
        printf("Micropro: Erase completed successfully\n");
    } else {
        printf("Micropro: Erase completed (no ACK)\n");
    }
    
    // Turn off voltage
    unsigned char voltage_off = 0x05;
    write(micropro_state.fd, &voltage_off, 1);
    usleep(50000);
    
    return SUCCESS;
}

// Micropro.exe compatible write
int micropro_write_block(unsigned int address, const unsigned char *data, int length)
{
    if (!micropro_state.is_connected || !micropro_state.device) return ERROR;
    
    DEBUG_PRINT("Micropro: Writing %d bytes to address 0x%04X\n", length, address);
    
    // Send write command
    unsigned char cmd = micropro_state.device->write_cmd;
    if (write(micropro_state.fd, &cmd, 1) != 1) {
        return ERROR;
    }
    
    // Send address (little-endian)
    unsigned char addr_bytes[2];
    addr_bytes[0] = address & 0xFF;
    addr_bytes[1] = (address >> 8) & 0xFF;
    if (write(micropro_state.fd, addr_bytes, 2) != 2) {
        return ERROR;
    }
    
    // Send length
    unsigned char len_byte = length;
    if (write(micropro_state.fd, &len_byte, 1) != 1) {
        return ERROR;
    }
    
    // Send data
    if (write(micropro_state.fd, data, length) != length) {
        return ERROR;
    }
    
    // Wait for write completion
    usleep(micropro_state.device->write_delay_ms * 1000);
    
    return SUCCESS;
}

// Micropro.exe compatible read
int micropro_read_block(unsigned int address, unsigned char *buffer, int length)
{
    if (!micropro_state.is_connected || !micropro_state.device) return ERROR;
    
    DEBUG_PRINT("Micropro: Reading %d bytes from address 0x%04X\n", length, address);
    
    // Send read command
    unsigned char cmd = micropro_state.device->read_cmd;
    if (write(micropro_state.fd, &cmd, 1) != 1) {
        return ERROR;
    }
    
    // Send address (little-endian)
    unsigned char addr_bytes[2];
    addr_bytes[0] = address & 0xFF;
    addr_bytes[1] = (address >> 8) & 0xFF;
    if (write(micropro_state.fd, addr_bytes, 2) != 2) {
        return ERROR;
    }
    
    // Send length
    unsigned char len_byte = length;
    if (write(micropro_state.fd, &len_byte, 1) != 1) {
        return ERROR;
    }
    
    // Read data with timeout
    int total_read = 0;
    int retries = 0;
    
    while (total_read < length && retries < 100) {
        int bytes = read(micropro_state.fd, buffer + total_read, length - total_read);
        if (bytes > 0) {
            total_read += bytes;
        } else {
            usleep(10000);
            retries++;
        }
    }
    
    return (total_read == length) ? SUCCESS : ERROR;
}

// High-level Micropro.exe compatible programming
int micropro_program_device(const char *device_name, const unsigned char *rom_data, 
                            int rom_size, unsigned int config_word)
{
    printf("Micropro: Programming %s with Micropro.exe protocol\n", device_name);
    
    // Initialize connection
    if (micropro_init_sequence() != SUCCESS) {
        fprintf(stderr, "Micropro: Initialization failed\n");
        return ERROR;
    }
    
    // Send device parameters
    if (micropro_send_device_params(device_name) != SUCCESS) {
        fprintf(stderr, "Micropro: Failed to set device parameters\n");
        return ERROR;
    }
    
    // Erase chip
    if (micropro_erase_chip() != SUCCESS) {
        fprintf(stderr, "Micropro: Erase failed\n");
        return ERROR;
    }
    
    // Turn on programming voltage
    unsigned char voltage_on = 0x04;
    write(micropro_state.fd, &voltage_on, 1);
    usleep(50000);
    
    // Write ROM in blocks
    int block_size = micropro_state.device->block_size;
    for (int offset = 0; offset < rom_size; offset += block_size) {
        int chunk = (rom_size - offset > block_size) ? block_size : (rom_size - offset);
        
        if (micropro_write_block(offset, rom_data + offset, chunk) != SUCCESS) {
            fprintf(stderr, "Micropro: Write failed at offset 0x%04X\n", offset);
            return ERROR;
        }
        
        printf("Micropro: Progress: %d/%d bytes (%.1f%%)\n", 
               offset + chunk, rom_size, (float)(offset + chunk) * 100.0 / rom_size);
    }
    
    // Write configuration word at 0x2007
    printf("Micropro: Writing configuration word 0x%04X\n", config_word);
    unsigned char config_data[2];
    config_data[0] = config_word & 0xFF;
    config_data[1] = (config_word >> 8) & 0xFF;
    
    unsigned char config_cmd[] = {0x09, 0x07, 0x20, config_data[0], config_data[1]};
    write(micropro_state.fd, config_cmd, 5);
    usleep(50000);
    
    // Turn off voltage
    unsigned char voltage_off = 0x05;
    write(micropro_state.fd, &voltage_off, 1);
    usleep(50000);
    
    printf("Micropro: Programming completed\n");
    
    // Verify
    printf("Micropro: Verifying...\n");
    unsigned char *verify_buffer = malloc(rom_size);
    if (verify_buffer) {
        int errors = 0;
        
        // Turn voltage back on for reading
        write(micropro_state.fd, &voltage_on, 1);
        usleep(50000);
        
        for (int offset = 0; offset < rom_size; offset += block_size) {
            int chunk = (rom_size - offset > block_size) ? block_size : (rom_size - offset);
            
            if (micropro_read_block(offset, verify_buffer + offset, chunk) != SUCCESS) {
                fprintf(stderr, "Micropro: Verify read failed at 0x%04X\n", offset);
                errors++;
                break;
            }
        }
        
        // Compare
        if (errors == 0) {
            for (int i = 0; i < rom_size; i++) {
                if (verify_buffer[i] != rom_data[i]) {
                    if (errors < 10) {
                        printf("Micropro: Verify error at 0x%04X: expected 0x%02X, got 0x%02X\n",
                               i, rom_data[i], verify_buffer[i]);
                    }
                    errors++;
                }
            }
        }
        
        free(verify_buffer);
        
        // Turn voltage off
        write(micropro_state.fd, &voltage_off, 1);
        
        if (errors > 0) {
            fprintf(stderr, "Micropro: Verification failed with %d errors\n", errors);
            return ERROR;
        }
        
        printf("Micropro: Verification successful\n");
    }
    
    return SUCCESS;
}

// Open Micropro connection
int micropro_open(const char *port_name)
{
    struct termios tty;
    
    micropro_state.fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (micropro_state.fd < 0) {
        fprintf(stderr, "Micropro: Cannot open %s: %s\n", port_name, strerror(errno));
        return ERROR;
    }
    
    // Configure for Micropro.exe compatibility (19200 8N1)
    memset(&tty, 0, sizeof(tty));
    tcgetattr(micropro_state.fd, &tty);
    
    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    
    tcsetattr(micropro_state.fd, TCSANOW, &tty);
    tcflush(micropro_state.fd, TCIOFLUSH);
    
    printf("Micropro: Port %s opened (Micropro.exe compatible mode)\n", port_name);
    return SUCCESS;
}

// Close Micropro connection
void micropro_close(void)
{
    if (micropro_state.fd >= 0) {
        // Clear DTR/RTS before closing
        micropro_dtr_rts_control(1, 1);
        
        close(micropro_state.fd);
        micropro_state.fd = -1;
        micropro_state.is_connected = 0;
        printf("Micropro: Connection closed\n");
    }
}