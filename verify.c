//-----------------------------------------------------------------------------
//
//	Verification functions for K150 programmer
//
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "picdev.h"

// Forward declarations for functions from main.c
extern unsigned short int GetPgmSize(const PIC_DEFINITION *picDevice);
extern unsigned short int GetPgmStart(const PIC_DEFINITION *picDevice);
extern unsigned short int GetDataSize(const PIC_DEFINITION *picDevice);
extern unsigned short int GetDataStart(const PIC_DEFINITION *picDevice);
#include "k150.h"
#include "parse.h"
#include "record.h"

//--------------------------------------------------------------------
// Verify program memory against hex file
//--------------------------------------------------------------------
bool DoVerifyPgm(const PIC_DEFINITION *picDevice, FILE *theFile)
{
    unsigned char *hex_buffer, *read_buffer;
    unsigned int nextAddr;
    unsigned short int size;
    unsigned char data;
    bool fileDone, fail = false;
    
    size = GetPgmSize(picDevice) * 2;  // Size in bytes

    if (!size) {
        fprintf(stderr, "Device %s has no program memory!\n", picDevice->name);
        return false;
    }

    // Allocate buffers
    hex_buffer = (unsigned char*)malloc(size);
    read_buffer = (unsigned char*)malloc(size);
    
    if (!hex_buffer || !read_buffer) {
        fprintf(stderr, "K150: Failed to allocate verification buffers\n");
        if (hex_buffer) free(hex_buffer);
        if (read_buffer) free(read_buffer);
        return false;
    }

    // Initialize buffers with 0xFF
    memset(hex_buffer, 0xFF, size);
    memset(read_buffer, 0xFF, size);

    // Read hex file data
    InitParse();
    fileDone = !GetNextByte(theFile, &nextAddr, &data);
    
    while (!fileDone) {
        if (nextAddr < size) {
            hex_buffer[nextAddr] = data;
        }
        fileDone = !GetNextByte(theFile, &nextAddr, &data);
    }

    // Read ROM from device
    printf("K150: Reading ROM for verification (%d bytes)\n", size);
    if (k150_read_rom(read_buffer, size) != 0) {
        fprintf(stderr, "K150: Failed to read ROM for verification\n");
        free(hex_buffer);
        free(read_buffer);
        return false;
    }

    // Compare data
    printf("K150: Comparing ROM data...\n");
    int mismatches = 0;
    for (int i = 0; i < size; i++) {
        if (hex_buffer[i] != read_buffer[i]) {
            if (mismatches < 10) {  // Show first 10 mismatches
                printf("K150: Mismatch at address 0x%04X: expected 0x%02X, read 0x%02X\n", 
                       i, hex_buffer[i], read_buffer[i]);
            }
            mismatches++;
            fail = true;
        }
    }

    if (fail) {
        printf("K150: ROM verification FAILED (%d mismatches)\n", mismatches);
    } else {
        printf("K150: ROM verification PASSED\n");
    }

    free(hex_buffer);
    free(read_buffer);
    return !fail;
}

//--------------------------------------------------------------------
// Verify EEPROM data against hex file
//--------------------------------------------------------------------
bool DoVerifyData(const PIC_DEFINITION *picDevice, FILE *theFile)
{
    unsigned char *hex_buffer, *read_buffer;
    unsigned short int size;
    unsigned int nextAddr;
    unsigned char data;
    bool fileDone, fail = false;

    size = GetDataSize(picDevice);

    if (!size) {
        fprintf(stderr, "Device %s has no EEPROM data!\n", picDevice->name);
        return false;
    }

    // Allocate buffers
    hex_buffer = (unsigned char*)malloc(size);
    read_buffer = (unsigned char*)malloc(size);
    
    if (!hex_buffer || !read_buffer) {
        fprintf(stderr, "K150: Failed to allocate EEPROM verification buffers\n");
        if (hex_buffer) free(hex_buffer);
        if (read_buffer) free(read_buffer);
        return false;
    }

    // Initialize buffers with 0xFF
    memset(hex_buffer, 0xFF, size);
    memset(read_buffer, 0xFF, size);

    // Read hex file data
    InitParse();
    fileDone = !GetNextByte(theFile, &nextAddr, &data);
    
    while (!fileDone) {
        if (nextAddr < size) {
            hex_buffer[nextAddr] = data;
        }
        fileDone = !GetNextByte(theFile, &nextAddr, &data);
    }

    // Read EEPROM from device
    printf("K150: Reading EEPROM for verification (%d bytes)\n", size);
    if (k150_read_eeprom(read_buffer, size) != 0) {
        fprintf(stderr, "K150: Failed to read EEPROM for verification\n");
        free(hex_buffer);
        free(read_buffer);
        return false;
    }

    // Compare data
    printf("K150: Comparing EEPROM data...\n");
    int mismatches = 0;
    for (int i = 0; i < size; i++) {
        if (hex_buffer[i] != read_buffer[i]) {
            if (mismatches < 10) {  // Show first 10 mismatches
                printf("K150: Mismatch at address 0x%04X: expected 0x%02X, read 0x%02X\n", 
                       i, hex_buffer[i], read_buffer[i]);
            }
            mismatches++;
            fail = true;
        }
    }

    if (fail) {
        printf("K150: EEPROM verification FAILED (%d mismatches)\n", mismatches);
    } else {
        printf("K150: EEPROM verification PASSED\n");
    }

    free(hex_buffer);
    free(read_buffer);
    return !fail;
}
