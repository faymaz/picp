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

#ifndef __K150_H_
#define __K150_H_

// K150 Protocol Commands (P18A)
#define K150_CMD_INIT_PIC        1
#define K150_CMD_PROGRAM_ROM     2
#define K150_CMD_PROGRAM_EEPROM  3
#define K150_CMD_PROGRAM_CONFIG  9
#define K150_CMD_READ_ROM        11
#define K150_CMD_READ_EEPROM     12
#define K150_CMD_READ_CONFIG     13
#define K150_CMD_ERASE_CHIP      15
#define K150_CMD_ERASE_CHECK_ROM 16
#define K150_CMD_GET_VERSION     21
#define K150_CMD_GET_PROTOCOL    22

// K150 Response codes
#define K150_RESP_OK             0x59
#define K150_RESP_ERROR          'N'
#define K150_RESP_CONFIG         'C'
#define K150_RESP_ACKNOWLEDGE    'A'

// K150 Programmer version
#define K150_VERSION             3

// Function declarations
int k150_open_port(char *port_name);
void k150_close_port(void);
int k150_is_port_open(void);
int k150_detect_programmer(void);
int k150_get_version(void);
int k150_get_protocol(char *protocol);
int k150_init_pic(int pic_type);
int k150_erase_chip(void);
int k150_program_rom(unsigned char *data, int size);
int k150_read_rom(unsigned char *data, int size);
int k150_verify_rom(unsigned char *programmed_data, int size);
int k150_program_eeprom(unsigned char *data, int size);
int k150_read_eeprom(unsigned char *data, int size);
int k150_program_config(unsigned int config_word);
int k150_read_config(unsigned int *config_word, unsigned int *device_id);
int k150_erase_check_rom(void);

// Internal helper functions
int k150_send_command(unsigned char cmd);
int k150_send_byte(unsigned char byte);
int k150_receive_byte(unsigned char *byte);
int k150_receive_response(void);

#endif // __K150_H_
