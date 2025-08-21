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
//
//	to do:
//		motorola S-records should be supported, too
//
//-----------------------------------------------------------------------------

#include <stdio.h>

#ifdef WIN32
#include	<windows.h>
#define	bool	int
#define	true	TRUE
#define	false	FALSE
#endif

#include "record.h"
#include	"picdev.h"

#define DATARECORD	0
#define ENDRECORD		1
#define EXTADDRESS	4
#define REC_LENGTH	16

//-----------------------------------------------------------------------------
// write an address record to the output file
static void DumpIntelHexExtendedAddressRecord(FILE *theFile, unsigned int address)
{
	int	checksum = 2 + EXTADDRESS + address + (address >> 8);

	fprintf(theFile, ":%02X%04X%02X%04X%02X\n",
		2, 0, EXTADDRESS, address, (-checksum) & 0xff);
}

//-----------------------------------------------------------------------------
// create a line of intel hex output to theFile
//  theFile -- file pointer to the output file
//  address -- the address of the record
//  theBytes -- pointer to the actual data
//  numBytes -- number of bytes in the record
static void DumpIntelHexLine(FILE *theFile, unsigned int address, unsigned char *theBytes, int numBytes, unsigned short int blankData)
{
	int	i, checkSum;
	unsigned short int	data;
	bool	skip = true;

	if (blankData)		// writing program data, not eeprom data
	{
		for (i=0; i<numBytes; i += 2)		// check for valid data in program space
		{
			data = (theBytes[i + 1] << 8) | (theBytes[i] & 0xff);

			if (data != blankData)			// non-blank data found, do not skip this record
			{
				skip = false;
				break;
			}
		}
	}
	else
		skip = false;		// always output lines for eeprom data

// If this line of program memory contains only blank values, do not write it.

	if (!skip)
	{
		fprintf(theFile, ":%02X%04X%02X", numBytes, address, DATARECORD);	// write the stub (colon, length of record, address, record type)
		checkSum = 0;

		for (i=0; i<numBytes; i++)
		{
			checkSum += theBytes[i];
			fprintf(theFile, "%02X", theBytes[i]);		// write a data byte as ASCII
		}

		checkSum += (address & 0xFF) + (address >> 8) + numBytes + DATARECORD;
		fprintf(theFile, "%02X\n", (-checkSum) & 0xFF);		// add the checksum and a line terminator
	}
}

//-----------------------------------------------------------------------------
// Dump an end of file record marker out to theFile
static void DumpIntelHexEOR(FILE *theFile)
{
	fprintf(theFile, ":00000001FF\n");			// provide end record
}

//-----------------------------------------------------------------------------
// write a hex record to the output file
// DEBUG should be able to specify intel hex or motorola S
void WriteHexRecord(FILE *outFile, unsigned char *theBuffer, unsigned int address, unsigned short int size, unsigned short int blankData)
{
	int				bytesLeft, numBytes;
	unsigned int	extendedAddress;
	
	bytesLeft = size;
	extendedAddress = address & 0xffff0000;

	DumpIntelHexExtendedAddressRecord(outFile, extendedAddress >> 16);

	while (bytesLeft)
	{
		numBytes = bytesLeft > REC_LENGTH ? REC_LENGTH : bytesLeft;

		if (((address + numBytes) & 0xffff0000) != extendedAddress)
			numBytes = extendedAddress - address;

		DumpIntelHexLine(outFile, address, &theBuffer[size - bytesLeft], numBytes, blankData);
		address += numBytes;
		bytesLeft -= numBytes;

		if (bytesLeft && ((address & 0xffff0000) != extendedAddress))
		{
			extendedAddress = address & 0xffff0000;
			DumpIntelHexExtendedAddressRecord(outFile, extendedAddress >> 16);
		}
	}

	DumpIntelHexEOR(outFile);
}
