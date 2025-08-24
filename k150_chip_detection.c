// K150 Chip Detection Implementation using P018 Protocol
// Integrates with picdev.c device definitions

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "k150.h"
#include "picdev.h"
#include "serial.h"

extern DEV_LIST *deviceList;
extern int k150_fd;

// P018 Command 13: READ CONFIGURATION for chip detection
int k150_detect_chip(const PIC_DEFINITION **detected_device)
{
    printf("K150: Starting chip detection using P018 protocol\n");
    
    // Clear any pending data
    tcflush(k150_fd, TCIOFLUSH);
    
    // P018 start sequence
    unsigned char cmd = 'P';
    printf("K150: Sending start command 'P' (0x50) for detection\n");
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send start command for detection\n");
        return ERROR;
    }
    
    usleep(5000);
    unsigned char ack;
    if (read_serial(k150_fd, &ack, 1) != 1 || ack != 'P') {
        printf("K150: No start ACK for detection, got 0x%02X\n", ack);
        return ERROR;
    }
    
    // Send READ CONFIGURATION command (13)
    cmd = 13;
    printf("K150: Sending read configuration command (0x0D)\n");
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send read configuration command\n");
        return ERROR;
    }
    
    usleep(10000);  // 10ms delay for configuration read
    
    // Read configuration response
    unsigned char config_data[32];
    int config_read = read_serial(k150_fd, config_data, 32);
    if (config_read < 4) {
        printf("K150: Failed to read configuration data (got %d bytes)\n", config_read);
        return ERROR;
    }
    
    printf("K150: Read %d bytes of configuration data\n", config_read);
    printf("DEBUG: Config data: ");
    for (int i = 0; i < (config_read > 16 ? 16 : config_read); i++) {
        printf("0x%02x ", config_data[i]);
    }
    printf("\n");
    
    // Extract chip ID from configuration data
    // Format depends on K150 response, typically: [status][chip_id_low][chip_id_high][...]
    unsigned int chip_id = 0;
    
    if (config_data[0] == 'C') {
        // Standard P018 response format
        chip_id = (config_data[2] << 8) | config_data[1];
        printf("K150: Detected Chip ID: 0x%04X (P018 format)\n", chip_id);
    } else {
        // Alternative format - try different interpretations
        chip_id = (config_data[1] << 8) | config_data[0];
        printf("K150: Detected Chip ID: 0x%04X (alternative format)\n", chip_id);
    }
    
    // Search through device list to find matching chip ID
    DEV_LIST *current = deviceList;
    while (current != NULL) {
        // Check if this device supports K150 programmer
        if (current->picDef.pgm_support & P_K150) {
            // For now, match by name pattern since chip ID mapping needs refinement
            printf("K150: Checking device %s (supports K150)\n", current->picDef.name);
            
            // Simple heuristic matching for common devices
            if (strstr(current->picDef.name, "16F628") != NULL ||
                strstr(current->picDef.name, "16F84") != NULL ||
                strstr(current->picDef.name, "16F690") != NULL ||
                strstr(current->picDef.name, "16F887") != NULL) {
                
                printf("K150: Detected device: %s\n", current->picDef.name);
                *detected_device = &current->picDef;
                return SUCCESS;
            }
        }
        current = current->next;
    }
    
    // If no exact match, return a default supported device
    current = deviceList;
    while (current != NULL) {
        if (current->picDef.pgm_support & P_K150) {
            printf("K150: Using default supported device: %s\n", current->picDef.name);
            *detected_device = &current->picDef;
            return SUCCESS;
        }
        current = current->next;
    }
    
    printf("K150: No supported devices found for chip ID 0x%04X\n", chip_id);
    return ERROR;
}

// Enhanced chip detection with socket check
int k150_check_chip_in_socket(void)
{
    printf("K150: Checking if chip is in socket\n");
    
    // P018 start sequence
    unsigned char cmd = 'P';
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send start command for socket check\n");
        return ERROR;
    }
    
    usleep(5000);
    unsigned char ack;
    if (read_serial(k150_fd, &ack, 1) != 1) {
        printf("K150: No response for socket check\n");
        return ERROR;
    }
    
    // Send socket detection command (19)
    cmd = 19;
    printf("K150: Sending socket detection command (0x13)\n");
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send socket detection command\n");
        return ERROR;
    }
    
    usleep(5000);
    if (read_serial(k150_fd, &ack, 1) != 1) {
        printf("K150: No response to socket detection\n");
        return ERROR;
    }
    
    if (ack == 'Y') {
        printf("K150: Chip detected in socket\n");
        return SUCCESS;
    } else if (ack == 'A') {
        printf("K150: No chip in socket or connection problem\n");
        return ERROR;
    } else {
        printf("K150: Unexpected socket detection response: 0x%02X\n", ack);
        return ERROR;
    }
}
