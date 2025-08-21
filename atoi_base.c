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
//
//	this interface to the PICSTART Plus programmer is provided in an effort
//	to make the PICSTART (and thus, the PIC family of microcontrollers) useful
//	and accessible across multiple platforms, not just one particularly well-
//	known, unstable one.
//
//-----------------------------------------------------------------------------
//
//	Copyright (C) 1999-2002 Cosmodog, Ltd.
// Copyright (c) 2004 Jeffery L. Post
//
// Please send bug reports for version 0.6.0 or higher to
// j_post <AT> pacbell <DOT> net.
//
//	This program is free software; you can redistribute it and/or
//	modify it under the terms of the GNU General Public License
//	as published by the Free Software Foundation; either version 2
//	of the License, or (at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program; if not, write to the Free Software
//	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//-----------------------------------------------------------------------------

#ifdef WIN32
#include	<windows.h>
#define	bool	int
#define	true	TRUE
#define	false	FALSE
#endif

#include	<stdio.h>
#include <ctype.h>
#include <limits.h>
#include <stdlib.h>

#include "atoi_base.h"

//-----------------------------------------------------------------------------
//
// convert string into an unsigned int, deciphering the base.  stop on terminator
//	or first bad character return true if result is okay, false if unable to
//	interpret the number
//
//  	0bnnnnnnnnnnnn = binary (or 0Bnnnnnnnnnnnn)
//		0xnnnn = hex (or 0Xnnnn)
//		anything else is decimal
//
//-----------------------------------------------------------------------------

bool atoi_base(char *str, unsigned int *result)
{
	int	base, digit;
	bool	fail;
		
	base = 10;							// assume decimal
	fail = false;

	*result = 0;						// start at zero

	if (*str)							// do nothing if it's a null string
	{
		if (*str == '0')					// figure out the base
		{
			str++;							// skip the zero

			if (toupper(*str) == 'X')
			{
				base = 16;					// leading 0x (or 0X), it's hex
				str++;						// skip the x
			}
			else if (toupper(*str) == 'B')
			{
				base = 2;					// leading 0b (or 0B), it's binary
				str++;						// skip the b
			}
			else
				base = 8;					// leading 0, assume octal
		}

		while (*str && !fail)
		{
			digit = toupper(*str++);	// force all upper case for hex digits
			digit = (digit >= 'A') ? digit - 'A' + 0x0a : digit - '0';	// convert to 0-9, A-F (or try to)

			if (digit >= 0 && digit < base)			// make sure digit can be represented in this base
				*result = *result * base + digit;	// shift up one order of magnitude, add in this digit
			else
				fail = true;				// bad character, fail
		}
	}

	return(!fail);
}

