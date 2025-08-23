//-----------------------------------------------------------------------------
//
//	PICSTART Plus programming interface
//
//-----------------------------------------------------------------------------
//
//	Cosmodog, Ltd.
//	415 West Huron Street
//	Chicago, IL   60610
//	http://www.cosmodog.com
//
// Maintained at
// http://home.pacbell.net/theposts/picmicro
//
//-----------------------------------------------------------------------------

#include <stdbool.h>

#ifndef __RECORD_H_
#define __RECORD_H_

void	WriteHexRecord(FILE *outFile, unsigned char *theBuffer, unsigned int address, unsigned short int size, unsigned short int blankData);

#endif // defined __RECORD_H_
