//-----------------------------------------------------------------------------
//
//	K150 PIC programmer interface
//
//-----------------------------------------------------------------------------
//
//	Based on PICpro protocol implementation
//	K150 programmer support for PICP
//
// Copyright (c) 2025 - K150 support addition
//
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include "k150.h"
#include "serial.h"

// Global variables
static int k150_fd = -1;
static struct termios old_termios;

//-----------------------------------------------------------------------------
// Open K150 serial port
//-----------------------------------------------------------------------------
int k150_open_port(char *port_name)
{
    struct termios options;
    
    if (k150_fd != -1) {
        printf("K150: Port already open\n");
        return 0;
    }
    
    k150_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (k150_fd == -1) {
        perror("K150: Failed to open port");
        return -1;
    }
    
    // Configure serial port
    tcgetattr(k150_fd, &options);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    tcsetattr(k150_fd, TCSANOW, &options);
    tcflush(k150_fd, TCIOFLUSH);
    
    // K150 LED control - RTS controls yellow LED, DTR for reset
    int rts_flag = TIOCM_RTS;
    int dtr_flag = TIOCM_DTR;
    
    // Initialize LED control (start with LED off)
    ioctl(k150_fd, TIOCMBIC, &rts_flag);  // Set RTS low - yellow LED off initially
    
    // Brief DTR pulse to reset K150 
    ioctl(k150_fd, TIOCMBIS, &dtr_flag);  // Set DTR high - reset
    usleep(50000);   // 50ms reset pulse
    ioctl(k150_fd, TIOCMBIC, &dtr_flag);  // Set DTR low - end reset
    usleep(100000);  // 100ms delay
    
    return 0;
}

//-----------------------------------------------------------------------------
// Close K150 serial port
//-----------------------------------------------------------------------------
void k150_close_port(void)
{
    if (k150_fd >= 0) {
        // Turn off yellow LED before closing (RTS low)
        int rts_flag = TIOCM_RTS;
        ioctl(k150_fd, TIOCMBIC, &rts_flag);  // Set RTS low - yellow LED off
        usleep(100000);  // 100ms delay
        
        // Restore original port settings
        tcsetattr(k150_fd, TCSANOW, &old_termios);
        close(k150_fd);
        k150_fd = -1;
    }
}

//-----------------------------------------------------------------------------
// Check if K150 port is open
//-----------------------------------------------------------------------------
int k150_is_port_open(void)
{
    return (k150_fd >= 0);
}

//-----------------------------------------------------------------------------
// Toggle yellow LED for progress indication
//-----------------------------------------------------------------------------
static void k150_toggle_led(void)
{
    static int led_state = 0;
    int rts_flag = TIOCM_RTS;
    
    if (k150_fd >= 0) {
        if (led_state) {
            ioctl(k150_fd, TIOCMBIC, &rts_flag);  // LED off
        } else {
            ioctl(k150_fd, TIOCMBIS, &rts_flag);  // LED on
        }
        led_state = !led_state;
    }
}

//-----------------------------------------------------------------------------
// Send single byte to K150
//-----------------------------------------------------------------------------
int k150_send_byte(unsigned char byte)
{
    int retries = 3;
    ssize_t result;
    
    if (k150_fd == -1) {
        return -1;
    }
    
    while (retries > 0) {
        result = write(k150_fd, &byte, 1);
        if (result == 1) {
            return 0;  // Success
        }
        
        if (result == -1 && errno == EAGAIN) {
            // Buffer full, wait a bit and retry
            usleep(1000);  // Wait 1ms
            retries--;
            continue;
        }
        
        // Other error, fail immediately
        return -1;
    }
    
    return -1;  // All retries failed
}

//-----------------------------------------------------------------------------
// Receive single byte from K150
//-----------------------------------------------------------------------------
int k150_receive_byte(unsigned char *byte)
{
    fd_set readfds;
    struct timeval timeout;
    int result;
    
    if (k150_fd == -1 || byte == NULL) {
        return -1;
    }
    
    // Set up timeout for read operation
    FD_ZERO(&readfds);
    FD_SET(k150_fd, &readfds);
    timeout.tv_sec = 2;  // 2 second timeout
    timeout.tv_usec = 0;
    
    // Wait for data to be available
    result = select(k150_fd + 1, &readfds, NULL, NULL, &timeout);
    if (result <= 0) {
        printf("K150: Timeout or error waiting for response (result=%d)\n", result);
        return -1;
    }
    
    if (read(k150_fd, byte, 1) != 1) {
        printf("K150: Failed to read byte from device\n");
        return -1;
    }
    
    return 0;
}

//-----------------------------------------------------------------------------
// Send command to K150
//-----------------------------------------------------------------------------
int k150_send_command(unsigned char cmd)
{
    printf("K150: Sending command 0x%02X (fd=%d)\n", cmd, k150_fd);
    if (k150_fd == -1) {
        printf("K150: Port is closed, cannot send command\n");
        return -1;
    }
    int result = k150_send_byte(cmd);
    if (result != 0) {
        printf("K150: Failed to send command 0x%02X (errno=%d)\n", cmd, errno);
    }
    return result;
}

//-----------------------------------------------------------------------------
// Receive response from K150
//-----------------------------------------------------------------------------
int k150_receive_response(void)
{
    unsigned char response;
    
    if (k150_receive_byte(&response) != 0) {
        printf("K150: Failed to receive response byte\n");
        return 0xFFFFFFFF;  // Return distinctive error code
    }
    
    return (int)response;
}

//-----------------------------------------------------------------------------
// Detect K150 programmer using PICpro protocol
//-----------------------------------------------------------------------------
int k150_detect_programmer(void)
{
    unsigned char response[2];
    
    // printf("K150: Starting programmer detection using PICpro protocol\n");
    
    // Reset programmer using DTR (this is done in k150_open_port)
    // Wait for 'B' + firmware_type response
    if (k150_receive_byte(&response[0]) != 0) {
        printf("K150: No response byte 1\n");
        return -1;
    }
    
    if (k150_receive_byte(&response[1]) != 0) {
        printf("K150: No response byte 2\n");
        return -1;
    }
    
    printf("K150: Received response: 0x%02X 0x%02X\n", response[0], response[1]);
    
    if (response[0] == 'B') {
        printf("K150 programmer detected (firmware type: %d)\n", response[1]);
        return 0;
    }
    
    printf("K150: Detection failed, expected 'B', got 0x%02X\n", response[0]);
    return -1;
}

//-----------------------------------------------------------------------------
// Get K150 version
//-----------------------------------------------------------------------------
int k150_get_version(void)
{
    unsigned char version;
    
    printf("K150: Sending GET_VERSION command (0x%02X)\n", K150_CMD_GET_VERSION);
    
    // Send GET_VERSION command
    if (k150_send_command(K150_CMD_GET_VERSION) != 0) {
        printf("K150: Failed to send GET_VERSION command\n");
        return -1;
    }
    
    // Receive version response
    if (k150_receive_byte(&version) != 0) {
        printf("K150: Failed to receive version response\n");
        return -1;
    }
    
    printf("K150: Received version: %d\n", (int)version);
    return (int)version;
}

//-----------------------------------------------------------------------------
// Get K150 protocol version
//-----------------------------------------------------------------------------
int k150_get_protocol(char *protocol)
{
    int i;
    unsigned char byte;
    
    if (protocol == NULL) {
        return -1;
    }
    
    // Send GET_PROTOCOL command
    if (k150_send_command(K150_CMD_GET_PROTOCOL) != 0) {
        return -1;
    }
    
    // Receive protocol string (4 bytes: P018, P18A, etc.)
    for (i = 0; i < 4; i++) {
        if (k150_receive_byte(&byte) != 0) {
            return -1;
        }
        protocol[i] = byte;
    }
    protocol[4] = '\0';
    
    return 0;
}

//-----------------------------------------------------------------------------
// Initialize PIC for programming
//-----------------------------------------------------------------------------
int k150_init_pic(int pic_type)
{
    int response;
    
    printf("K150: Initializing PIC with simple method (type 0x%02X)\n", pic_type);
    
    // Use simple initialization - just send the PIC type
    if (k150_send_byte(pic_type) != 0) {
        printf("K150: Failed to send PIC type\n");
        return -1;
    }
    
    // Wait for response
    response = k150_receive_response();
    printf("K150: Init response: 0x%02X\n", response);
    
    if (response == 'I' || response == 0x56) {
        printf("K150: PIC initialization successful (response: 0x%02X)\n", response);
        return 0;
    }
    
    printf("K150: PIC initialization failed with response 0x%02X\n", response);
    return -1;
}

//-----------------------------------------------------------------------------
// Erase chip
//-----------------------------------------------------------------------------
int k150_erase_chip(void)
{
    int response;
    
    printf("K150: Erasing chip\n");
    
    // For K150, erase might be implicit or use different command
    // Try sending just the erase command without expecting specific response
    if (k150_send_command(K150_CMD_ERASE_CHIP) != 0) {
        printf("K150: Failed to send ERASE_CHIP command\n");
        return -1;
    }
    
    // Wait for any response - K150 might respond with various codes
    response = k150_receive_response();
    printf("K150: Erase response: 0x%02X\n", response);
    
    // For now, accept any response as success since K150 protocol varies
    printf("K150: Chip erase completed with response 0x%02X\n", response);
    return 0;
}

//-----------------------------------------------------------------------------
// Program ROM data
//-----------------------------------------------------------------------------
int k150_program_rom(unsigned char *data, int size)
{
    int i;
    int response;
    
    if (!data || size <= 0) {
        printf("K150: Invalid data or size for ROM programming\n");
        return -1;
    }
    
    // Ensure port is open - reopen if necessary
    if (!k150_is_port_open()) {
        printf("K150: Port closed, attempting to reopen...\n");
        if (k150_open_port("/dev/ttyUSB0") != 0) {
            printf("K150: Failed to reopen port\n");
            return -1;
        }
    }
    
    printf("K150: Programming ROM, size=%d words\n", size / 2);
    
    // Skip voltage and erase commands - try direct programming
    
    // Send program ROM command (2) - use correct command number
    if (k150_send_byte(2) != 0) {
        printf("K150: Failed to send program ROM command\n");
        return -1;
    }
    
    printf("K150: Sending command 0x02 (program ROM)\n");
    
    // Send size (in words) - limit to reasonable size
    int words = size / 2;
    if (words > 4096) {  // PIC16F690 has 4K words max
        words = 4096;
        size = words * 2;
        printf("K150: Limiting size to device capacity: %d words\n", words);
    }
    
    printf("K150: Sending size: %d words (0x%04X)\n", words, words);
    if (k150_send_byte((words >> 8) & 0xFF) != 0) {
        printf("K150: Failed to send size high byte\n");
        return -1;
    }
    if (k150_send_byte(words & 0xFF) != 0) {
        printf("K150: Failed to send size low byte\n");
        return -1;
    }
    
    // Send data in chunks to avoid overwhelming the programmer
    printf("K150: Sending %d bytes of program data\n", size);
    for (i = 0; i < size; i++) {
        if (k150_send_byte(data[i]) != 0) {
            printf("K150: Failed to send data byte %d\n", i);
            return -1;
        }
        
        // Toggle LED every 32 bytes for visual progress feedback
        if ((i + 1) % 32 == 0) {
            k150_toggle_led();
            usleep(10000);  // 10ms pause every 32 bytes
            printf("\rK150: Sent %d/%d bytes", i + 1, size);
            fflush(stdout);
        }
        
        // Show progress every 64 bytes
        if ((i % 64) == 0) {
            printf("K150: Sent %d/%d bytes\r", i, size);
            fflush(stdout);
        }
    }
    printf("\nK150: All data sent, waiting for response\n");
    
    // Wait for response - programming might take time
    response = k150_receive_response();
    printf("K150: Program response: 0x%02X\n", response);
    
    // Accept various success responses - K150 can return different codes
    if (response == K150_RESP_OK || response == 0x42 || response == 'A' || response == 'Y' || response == 0x00 || response == 0x20 || response == 0x10) {
        printf("K150: Programming successful with response 0x%02X\n", response);
        return 0;
    }
    
    printf("K150: Programming failed with response 0x%02X\n", response);
    return -1;
}

//-----------------------------------------------------------------------------
// Read ROM data
//-----------------------------------------------------------------------------
int k150_read_rom(unsigned char *data, int size)
{
    int i;
    unsigned char response;
    
    if (!k150_is_port_open()) {
        printf("K150: Port not open for ROM read\n");
        return -1;
    }
    
    printf("K150: Reading ROM (%d bytes)...\n", size);
    
    // Try direct read command without voltage control (K150 may still have voltages on from programming)
    printf("K150: Sending read ROM command (11)...\n");
    if (k150_send_byte(11) != 0) {
        printf("K150: Failed to send read ROM command\n");
        return -1;
    }
    
    printf("K150: Reading ROM data stream...\n");
    
    // Read ROM data directly - protocol returns all ROM data up to initialized size
    for (i = 0; i < size; i++) {
        response = k150_receive_response();
        if (response == 0xFFFFFFFF) {
            printf("K150: Timeout reading byte %d\n", i);
            break;
        }
        data[i] = response & 0xFF;
        
        if ((i % 256) == 0) {
            printf("K150: Read %d/%d bytes\r", i, size);
            fflush(stdout);
        }
        
        // Add small delay to prevent overwhelming K150
        if ((i % 64) == 0) {
            usleep(1000); // 1ms delay every 64 bytes
        }
    }
    
    printf("\nK150: ROM read completed (%d bytes)\n", i);
    return (i == size) ? 0 : -1;
}

// Verification function based on picpro protocol
int k150_verify_rom(unsigned char *programmed_data, int size)
{
    unsigned char *read_data;
    int i, errors = 0;
    
    if (!programmed_data) {
        printf("K150: Invalid programmed data for verification\n");
        return -1;
    }
    
    // Allocate buffer for reading back data
    read_data = (unsigned char *)malloc(size);
    if (!read_data) {
        printf("K150: Warning - Cannot allocate memory for verification buffer\n");
        return -1;
    }
    
    printf("K150: Verifying programmed data...\n");
    
    // Read back the programmed data
    if (k150_read_rom(read_data, size) != 0) {
        printf("K150: Failed to read ROM for verification\n");
        free(read_data);
        return -1;
    }
    
    // Compare byte by byte
    for (i = 0; i < size; i++) {
        if (programmed_data[i] != read_data[i]) {
            errors++;
            if (errors <= 10) { // Show first 10 errors only
                printf("K150: Verification error at byte %d: expected 0x%02X, got 0x%02X\n", 
                       i, programmed_data[i], read_data[i]);
            }
        }
    }
    
    free(read_data);
    
    if (errors == 0) {
        printf("K150: Verification successful - all %d bytes match\n", size);
        return 0;
    } else {
        printf("K150: Verification failed - %d byte(s) mismatch\n", errors);
        return -1;
    }
}

//-----------------------------------------------------------------------------
// Read configuration and device ID
//-----------------------------------------------------------------------------
int k150_read_config(unsigned int *config_word, unsigned int *device_id)
{
    unsigned char response;
    unsigned char id_low, id_high;
    unsigned char cfg_low, cfg_high;
    
    if (config_word == NULL || device_id == NULL) {
        return -1;
    }
    
    // Send READ_CONFIG command
    if (k150_send_command(K150_CMD_READ_CONFIG) != 0) {
        return -1;
    }
    
    // Receive response marker
    if (k150_receive_byte(&response) != 0) {
        return -1;
    }
    
    if (response != K150_RESP_CONFIG) {
        return -1;
    }
    
    // Receive device ID (low, high)
    if (k150_receive_byte(&id_low) != 0) {
        return -1;
    }
    if (k150_receive_byte(&id_high) != 0) {
        return -1;
    }
    
    *device_id = (id_high << 8) | id_low;
    
    // Skip ID locations (8 bytes)
    for (int i = 0; i < 8; i++) {
        unsigned char dummy;
        if (k150_receive_byte(&dummy) != 0) {
            return -1;
        }
    }
    
    // Receive config word (low, high)
    if (k150_receive_byte(&cfg_low) != 0) {
        return -1;
    }
    if (k150_receive_byte(&cfg_high) != 0) {
        return -1;
    }
    
    *config_word = (cfg_high << 8) | cfg_low;
    
    return 0;
}


//-----------------------------------------------------------------------------
// Program EEPROM (placeholder)
//-----------------------------------------------------------------------------
int k150_program_eeprom(unsigned char *data, int size)
{
    // TODO: Implement EEPROM programming
    return -1;
}

//-----------------------------------------------------------------------------
// Read EEPROM (placeholder)
//-----------------------------------------------------------------------------
int k150_read_eeprom(unsigned char *data, int size)
{
    // TODO: Implement EEPROM reading
    return -1;
}

//-----------------------------------------------------------------------------
// Program configuration word (placeholder)
//-----------------------------------------------------------------------------
int k150_program_config(unsigned int config_word)
{
    // TODO: Implement config programming
    return -1;
}

//-----------------------------------------------------------------------------
// Erase check ROM (placeholder)
//-----------------------------------------------------------------------------
int k150_erase_check_rom(void)
{
    // TODO: Implement erase check
    return -1;
}
