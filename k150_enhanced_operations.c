// K150 Enhanced Operations using picdev.c integration
// Implements erase, program, and verify functions for all supported PIC devices

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include "k150.h"
#include "picdev.h"
#include "serial.h"

extern int k150_fd;
extern DEV_LIST *deviceList;

// P018 Protocol enhanced erase function
int k150_erase_chip_enhanced(const PIC_DEFINITION *picDevice)
{
    printf("K150: Starting chip erase for %s\n", picDevice->name);
    
    // Clear any pending data
    tcflush(k150_fd, TCIOFLUSH);
    
    // P018 protocol sequence for erase
    if (k150_init() != SUCCESS) {
        printf("K150: P018 start command failed for erase\n");
        return ERROR;
    }
    
    if (k150_send_init_params(picDevice) != SUCCESS) {
        printf("K150: P018 initialization failed for erase\n");
        return ERROR;
    }
    
    if (k150_voltages_on() != SUCCESS) {
        printf("K150: P018 voltages ON failed for erase\n");
        return ERROR;
    }
    
    // Send ERASE CHIP command (15)
    unsigned char cmd = 15;
    printf("K150: Sending erase chip command (0x0F) for %s\n", picDevice->name);
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send erase command\n");
        k150_voltages_off();
        return ERROR;
    }
    
    printf("DEBUG: write_serial sent 1 bytes: 0x0F\n");
    usleep(50000);  // 50ms delay for erase operation
    
    unsigned char ack;
    if (read_serial(k150_fd, &ack, 1) != 1) {
        printf("K150: No ACK received for erase command\n");
        k150_voltages_off();
        return ERROR;
    }
    
    // K150 may respond with different ACK codes for erase
    if (ack == 'Y' || ack == 'P' || ack == 0x59 || ack == 0x50 || ack == 0x03) {
        printf("K150: Erase ACK received (0x%02X)\n", ack);
        printf("DEBUG: read_serial got 1 bytes: 0x%02x\n", ack);
    } else {
        printf("K150: Unexpected erase ACK, got 0x%02X (continuing)\n", ack);
    }
    
    k150_voltages_off();
    printf("K150: Chip erase completed for %s\n", picDevice->name);
    return SUCCESS;
}

// P018 Protocol enhanced program function
int k150_program_rom_enhanced(const PIC_DEFINITION *picDevice, const unsigned char *data, int size)
{
    printf("K150: Programming %d bytes to %s\n", size, picDevice->name);
    
    // Clear any pending data
    tcflush(k150_fd, TCIOFLUSH);
    
    // P018 protocol sequence for programming
    if (k150_init() != SUCCESS) {
        printf("K150: P018 start command failed for program\n");
        return ERROR;
    }
    
    if (k150_send_init_params(picDevice) != SUCCESS) {
        printf("K150: P018 initialization failed for program\n");
        return ERROR;
    }
    
    if (k150_voltages_on() != SUCCESS) {
        printf("K150: P018 voltages ON failed for program\n");
        return ERROR;
    }
    
    // Send PROGRAM ROM command (7)
    unsigned char cmd = 7;
    printf("K150: Sending program ROM command (0x07) for %s\n", picDevice->name);
    if (write_serial(k150_fd, &cmd, 1) != 1) {
        printf("K150: Failed to send program command\n");
        k150_voltages_off();
        return ERROR;
    }
    
    printf("DEBUG: write_serial sent 1 bytes: 0x07\n");
    usleep(5000);  // 5ms delay
    
    unsigned char ack;
    if (read_serial(k150_fd, &ack, 1) != 1) {
        printf("K150: No ACK received for program start\n");
        k150_voltages_off();
        return ERROR;
    }
    
    // Check for program start ACK
    if (ack == 'Y' || ack == 'P' || ack == 0x59 || ack == 0x50) {
        printf("K150: Program start ACK received (0x%02X)\n", ack);
    } else {
        printf("K150: Unexpected program start ACK, got 0x%02X\n", ack);
        k150_voltages_off();
        return ERROR;
    }
    
    // Send data in chunks
    int total_written = 0;
    int chunk_size = 64;  // P018 standard chunk size
    
    while (total_written < size) {
        int remaining = size - total_written;
        int current_chunk = (remaining > chunk_size) ? chunk_size : remaining;
        
        if (write_serial(k150_fd, data + total_written, current_chunk) != current_chunk) {
            printf("K150: Program write failed at byte %d\n", total_written);
            k150_voltages_off();
            return ERROR;
        }
        
        total_written += current_chunk;
        printf("K150: Program progress: %d/%d bytes (%.1f%%)\n", 
               total_written, size, (total_written * 100.0) / size);
        
        usleep(10000);  // 10ms between chunks
    }
    
    // Wait for program completion ACK
    usleep(50000);  // 50ms for final programming
    if (read_serial(k150_fd, &ack, 1) != 1) {
        printf("K150: No final ACK received for program completion\n");
        k150_voltages_off();
        return ERROR;
    }
    
    if (ack == 'P' || ack == 'Y' || ack == 0x50 || ack == 0x59 || ack == 0x10) {
        printf("K150: Program completion ACK received (0x%02X)\n", ack);
    } else {
        printf("K150: Unexpected program completion ACK, got 0x%02X\n", ack);
    }
    
    k150_voltages_off();
    printf("K150: Programming completed for %s\n", picDevice->name);
    return SUCCESS;
}

// Enhanced verify function
int k150_verify_rom_enhanced(const PIC_DEFINITION *picDevice, const unsigned char *expected_data, int size)
{
    printf("K150: Verifying %d bytes from %s\n", size, picDevice->name);
    
    // Allocate buffer for read data
    unsigned char *read_data = malloc(size);
    if (!read_data) {
        printf("K150: Failed to allocate memory for verification\n");
        return ERROR;
    }
    
    // Read ROM using enhanced function
    int result = k150_read_rom_with_device(picDevice, read_data, size);
    if (result != SUCCESS) {
        printf("K150: Verification failed - unable to read ROM from %s\n", picDevice->name);
        free(read_data);
        return ERROR;
    }
    
    // Compare data byte by byte
    int mismatch_count = 0;
    int first_mismatch = -1;
    
    for (int i = 0; i < size; i++) {
        if (read_data[i] != expected_data[i]) {
            if (first_mismatch == -1) {
                first_mismatch = i;
            }
            mismatch_count++;
            
            // Show first few mismatches for debugging
            if (mismatch_count <= 10) {
                printf("K150: Mismatch at byte %d: expected 0x%02X, got 0x%02X\n",
                       i, expected_data[i], read_data[i]);
            }
        }
    }
    
    free(read_data);
    
    if (mismatch_count == 0) {
        printf("K150: Verification successful - %s programmed correctly\n", picDevice->name);
        return SUCCESS;
    } else {
        printf("K150: Verification failed - %d byte(s) mismatch in %s (first at byte %d)\n",
               mismatch_count, picDevice->name, first_mismatch);
        return ERROR;
    }
}

// Multi-device operation dispatcher
int k150_perform_operation(const char *device_name, const char *operation, const char *filename)
{
    printf("K150: Performing %s operation on %s\n", operation, device_name);
    
    // Find device in picdev.c list
    const PIC_DEFINITION *picDevice = NULL;
    DEV_LIST *current = deviceList;
    
    while (current != NULL) {
        if (strcasecmp(current->picDef.name, device_name) == 0 && 
            (current->picDef.pgm_support & P_K150)) {
            picDevice = &current->picDef;
            break;
        }
        current = current->next;
    }
    
    if (!picDevice) {
        printf("K150: Device %s not found or not supported by K150\n", device_name);
        return ERROR;
    }
    
    // Calculate ROM size from device definition
    int pgm_size = (picDevice->def[PD_PGM_SIZEH] << 8) | picDevice->def[PD_PGM_SIZEL];
    int rom_bytes = pgm_size * 2;  // Convert words to bytes
    
    printf("K150: Device %s - ROM size: %d words (%d bytes)\n", 
           picDevice->name, pgm_size, rom_bytes);
    
    if (strcmp(operation, "erase") == 0) {
        return k150_erase_chip_enhanced(picDevice);
    }
    else if (strcmp(operation, "read") == 0) {
        unsigned char *buffer = malloc(rom_bytes);
        if (!buffer) {
            printf("K150: Failed to allocate memory for read operation\n");
            return ERROR;
        }
        
        int result = k150_read_rom_with_device(picDevice, buffer, rom_bytes);
        if (result == SUCCESS && filename) {
            // Save to hex file
            FILE *fp = fopen(filename, "w");
            if (fp) {
                // Write Intel HEX format
                fprintf(fp, ":020000040000FA\n");  // Extended address record
                for (int i = 0; i < rom_bytes; i += 16) {
                    int line_bytes = (rom_bytes - i > 16) ? 16 : rom_bytes - i;
                    int checksum = line_bytes + (i >> 8) + (i & 0xFF);
                    
                    fprintf(fp, ":%02X%04X00", line_bytes, i);
                    for (int j = 0; j < line_bytes; j++) {
                        fprintf(fp, "%02X", buffer[i + j]);
                        checksum += buffer[i + j];
                    }
                    fprintf(fp, "%02X\n", (0x100 - (checksum & 0xFF)) & 0xFF);
                }
                fprintf(fp, ":00000001FF\n");  // End of file record
                fclose(fp);
                printf("K150: Data saved to %s\n", filename);
            }
        }
        
        free(buffer);
        return result;
    }
    else if (strcmp(operation, "program") == 0 && filename) {
        // Read hex file and program
        FILE *fp = fopen(filename, "r");
        if (!fp) {
            printf("K150: Cannot open %s for reading\n", filename);
            return ERROR;
        }
        
        unsigned char *buffer = calloc(rom_bytes, 1);
        if (!buffer) {
            printf("K150: Failed to allocate memory for program operation\n");
            fclose(fp);
            return ERROR;
        }
        
        // Simple Intel HEX parser
        char line[256];
        while (fgets(line, sizeof(line), fp)) {
            if (line[0] != ':') continue;
            
            int byte_count, address, record_type;
            sscanf(line + 1, "%02X%04X%02X", &byte_count, &address, &record_type);
            
            if (record_type == 0) {  // Data record
                for (int i = 0; i < byte_count; i++) {
                    int byte_val;
                    sscanf(line + 9 + i * 2, "%02X", &byte_val);
                    if (address + i < rom_bytes) {
                        buffer[address + i] = byte_val;
                    }
                }
            }
        }
        fclose(fp);
        
        // Program and verify
        int result = k150_program_rom_enhanced(picDevice, buffer, rom_bytes);
        if (result == SUCCESS) {
            result = k150_verify_rom_enhanced(picDevice, buffer, rom_bytes);
        }
        
        free(buffer);
        return result;
    }
    else {
        printf("K150: Unknown operation: %s\n", operation);
        return ERROR;
    }
}
