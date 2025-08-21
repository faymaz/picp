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

#ifndef __PARSE_H_
#define __PARSE_H_

bool	GetNextByte(FILE *theFile, unsigned int *address, unsigned char *data);
void	InitParse();

#endif // defined __PARSE_H_
