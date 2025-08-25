//-----------------------------------------------------------------------------
//
//	picpro Backend Integration Header
//	Provides compatibility with epk150.hex firmware via picpro
//
//-----------------------------------------------------------------------------

#ifndef __PICPRO_BACKEND_H_
#define __PICPRO_BACKEND_H_

// Function declarations for picpro backend integration
int picpro_check_availability(void);
int picpro_detect_chip(const char *port, const char *expected_chip);
int picpro_program_chip(const char *port, const char *chip_type, const char *hex_file);
int picpro_read_chip(const char *port, const char *chip_type, const char *output_file);
int picpro_erase_chip(const char *port, const char *chip_type);

#endif // __PICPRO_BACKEND_H_
