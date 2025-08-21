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

#ifndef __ATOI_BASE_H_
#define __ATOI_BASE_H_

#ifdef WIN32
#define	bool	int
#endif

bool	atoi_base(char *str, unsigned int *result);

#endif // defined __ATOI_BASE_H_

