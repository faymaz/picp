//-----------------------------------------------------------------------------
//
//	K150 PIC programmer interface
//
//-----------------------------------------------------------------------------
//
//	K150 programmer support for PICP
//	Reverse-engineered protocol implementation
//
// Copyright (c) 2025 - K150 support addition
//
//-----------------------------------------------------------------------------

#ifndef __K150_H_
#define __K150_H_

#include <stdbool.h>  // for bool type

// K150 Protocol Commands (P18A)
#define K150_CMD_INIT_PIC        1
#define K150_CMD_PROGRAM_ROM     2
#define K150_CMD_PROGRAM_EEPROM  3
#define K150_CMD_PROGRAM_ID      4
#define K150_CMD_PROGRAM_CONFIG  9
#define K150_CMD_READ_ROM        11
#define K150_CMD_READ_EEPROM     12
#define K150_CMD_READ_ID         13
#define K150_CMD_READ_CONFIG     14
#define K150_CMD_ERASE_CHIP      15
#define K150_CMD_ERASE_CHECK_ROM 16
#define K150_CMD_GET_CHIP_ID     20
#define K150_CMD_GET_VERSION     21
#define K150_CMD_GET_PROTOCOL    22

// K150 Response codes
#define K150_RESP_OK             0x59
#define K150_RESP_ERROR          'N'
#define K150_RESP_CONFIG         'C'
#define K150_RESP_ACKNOWLEDGE    'A'

// Microbrn.exe uyumlu komutlar
#define K150_CMD_INIT            0x05  // Microbrn init komutu
#define K150_CMD_EXIT            0x01  // Microbrn exit komutu
#define K150_CMD_ACK             0x76  // Microbrn ACK yanıtı

// P18A Protocol commands (modern K150 firmware)
#define K150_P18A_DETECT         0x42
#define K150_P18A_ENTER_PROG     0x50
#define K150_P18A_EXIT_PROG      0x51
#define K150_P18A_READ_BLOCK     0x46
#define K150_P18A_WRITE_BLOCK    0x47
#define K150_P18A_ERASE_CHIP     0x4D
#define K150_P18A_ICSP_ENABLE    0x49

// Firmware version detection
#define K150_FW_P014             0x01
#define K150_FW_P016             0x02
#define K150_FW_P018             0x03
#define K150_FW_P18A             0x12

// K150 Programmer version
#define K150_VERSION             3

// Function declarations
int k150_open_port(const char *device);
int k150_close_port(void);
int k150_detect(void);
int k150_init_pic(void);
int k150_detect_programmer(void);
int k150_read_serial_quick(unsigned char *buf, int len);  // for quick protocol testing
int k150_is_port_open(void);
int k150_erase_chip(void);
int k150_read_rom(unsigned char *data, int size);
int k150_read_eeprom(unsigned char *data, int size);
int k150_program_rom(unsigned char *data, int size);
int k150_program_eeprom(unsigned char *data, int size);
int k150_verify_rom(unsigned char *expected, int size);
int k150_verify_eeprom(unsigned char *expected, int size);
int k150_read_config(unsigned char *config);
int k150_program_config(unsigned char *config);

// Enhanced functions using picdev.c integration
#include "picdev.h"
int k150_read_rom_with_device(const PIC_DEFINITION *picDevice, unsigned char *data, int size);
int k150_erase_chip_enhanced(const PIC_DEFINITION *picDevice);
int k150_program_rom_enhanced(const PIC_DEFINITION *picDevice, const unsigned char *data, int size);
int k150_verify_rom_enhanced(const PIC_DEFINITION *picDevice, const unsigned char *expected_data, int size);
int k150_perform_operation(const char *device_name, const char *operation, const char *filename);

// Serial port functions
int init_serial(const char *device);
int check_programmer(void);

// Chip detection functions
struct pic_device;
int k150_detect_chip(struct pic_device **detected_device);
int k150_detect_chip_with_type(const char *expected_chip);
int k150_check_chip_in_socket(void);
int k150_detect_chip_command_line(void);

// Internal helper functions
int k150_send_command(unsigned char cmd);
int k150_send_byte(unsigned char byte);
int k150_receive_byte(unsigned char *byte);
int k150_receive_response(void);

// Serial communication functions
int k150_write_serial(const unsigned char *buf, int len);
int k150_read_serial(unsigned char *buf, int len);

// Firmware version detection
int k150_get_firmware_version(void);
PIC_DEFINITION *k150_get_current_device(void);

// Configuration memory functions (fuse bit programming)
int k150_read_config(unsigned char *config);
int k150_program_config(unsigned char *config);
int k150_write_config_with_verify(unsigned int config_value);
int k150_parse_fuse_string(const char *fuse_string, const char *device_name, unsigned int *config_value);
int k150_read_config_to_hex(const char *filename, unsigned int config_addr);

// Utility functions
int k150_force_led_off(const char *device_path);

// Global variables
extern int theDevice;
extern bool isK150;  // K150 programmer active flag

#endif // __K150_H_
