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
//	This module reads the input file, autodetects the file type, and returns
//	bytes as requested.  Valid file formats are Intel Hex and Motorola
//	S-record.  Intel Hex is identified by a leading : on each line; Motorola S
//	is identified by a leading S on each line.  Anything else is an error.
//
//	Call InitParse() once to initialize, then call GetNextByte() to get each
//	character.  GetNextByte() will return false when out of characters.
//
//-----------------------------------------------------------------------------
//
// INTEL HEX:
//	:nnaaaarrdddddd...cc
//		where
//			nn   - number of data bytes
//			aaaa - address
//			rr   - record type (0 = data record, 1 = end record, 2 = segment address, 4 = extended address)
//			dd   - data bytes
//			cc   - checksum (0 minus the sum of all other bytes excluding the colon)
//
//	example:
//		data record:
//			:1000A0006707610AAE02EE046707610A0D028E005F
//
//			number of data bytes: 0x10 (16)
//			address: 0x00A0
//			record type: 0x00 (data record)
//			data: 0x67 0x07 0x61 0x0A 0xAE 0x02 0xEE 0x04 0x67 0x07 0x61 0x0A 0x0D 0x02 0x8E 0x00
//			checksum: 0x5f
//
//		end record:
//			:00000001FF
//
//			number of data bytes: 0x00
//			address: 0x0000 (unused)
//			record type: 0x01 (end record)
//			data: none
//			checksum: 0xff
//
//	the checksum is set such that the sum of all bytes (excluding the colon) equals zero
//	the end record is treated as end of segment, and does not terminate processing
//
//-----------------------------------------------------------------------------
//
// MOTOROLA S:
//	Sxnnaaaadddddd...cc
//		where:
//			Sx   - the record type
//			nn   - the number of data bytes
//			aaaa - address (aaaa for S1, aaaaaa for S2, aaaaaa for S3)
//			dd   - data bytes
//			cc   - checksum
//
//	S0: comment record
//	S1: data record with 16 bit address
//	S2: data record with 24 bit address
//	S3: data record with 32 bit address
//	S7: end of file record?
//	S9: end of file record
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

#include "parse.h"

#define	INTEL_CHAR		':'
#define	MOT_CHAR			'S'
#define	COMMENT_CHAR	';'
#define	SPACE_CHAR		' '
#define	NEWLINE_CHAR	'\n'
#define	NULL_CHAR		0

#define	DATARECORD	0				// data record
#define	ENDRECORD	1				// end record
#define	SEGADDRESS	2				// segment address record (INHX32)
#define	EXTADDRESS	4				// extended linear address record (INHX32)

#define MAX_LINE_LEN	256

static char	lineBuffer[MAX_LINE_LEN];	// place to read records from the input file

static int	byteCount;				// number of bytes read in and ready to be returned
static int	byteIdx;					// index to the next byte to be read

static unsigned int	byteAddress;		// address of the next byte to be read
static unsigned int	extendedLinearAddress;	// bits 31-16 are bits 31-16 of the address for intel hex records, bits 15-0 are 0

//-----------------------------------------------------------------------------
// convert two ascii hex characters to an 8-bit unsigned int
//  return zero if out of range (not ascii hex)
static unsigned char atoh(char high, char low)
{
	char	hi, lo;
	int	value;

	hi = toupper(high);
	lo = toupper(low);
	value = 0;

	if (hi >= '0' && hi <= '9')
		value += ((hi - '0') << 4);
	else if (hi >= 'A' && hi <= 'F')
		value += ((hi - 'A' + 0x0a) << 4);

	if (lo >= '0' && lo <= '9')
		value += (lo - '0');
	else if (lo >= 'A' && lo <= 'F')
		value += (lo - 'A' + 0x0a);

	return(value);
}

//-----------------------------------------------------------------------------
// read a line from the given file into the array line
// if there is a read error, return false
// lineLength is considered to be the maximum number of characters
// to read into the line (including the 0 terminator)
// if the line fills before a CR is hit, overFlow will be set true, and the rest of the
// line will be read, but not stored
// if the EOF is hit, atEOF is set true
static bool GetLine(FILE *theFile, char *line, int lineLength, bool *atEOF, bool *overFlow)
{
	int				i;
	unsigned char	c;
	unsigned int	numRead;
	bool				stopReading, hadError;

	i = 0;											// index into output line
	stopReading = hadError = (*atEOF) = (*overFlow)= false;

	while (!stopReading)
	{
		numRead = fread(&c, 1, 1, theFile);		// get character from input

		if (numRead)									// if something was read, check it, and add to output line
		{
			if (c == '\n' || c== '\0')				// found termination?
				stopReading = true;					// yes, output line is complete
			else
			{
				if (c != '\n' && c != '\r')		// see if the character should be added to the line
				{
					if (i < lineLength - 1)			// make sure there is room to store it
						line[i++] = c;					// store it if there is room
					else
						(*overFlow) = true;			// complain of overflow, but continue to the end
				}
			}
		}
		else
		{
			stopReading = true;

			if (feof(theFile))
				(*atEOF) = true;						// tell caller that EOF was encountered
			else
				hadError = true;
		}
	}

	if (lineLength)
		line[i] = '\0';								// terminate the line if possible

	return(!hadError);								// return error status
}

//-----------------------------------------------------------------------------
// set up to interpret the line as an intel hex record
static bool GetIntelRecord(FILE *theFile)
{
	bool	done;
	int	i, len, type, chksum;
		
	done = false;

	type = atoh(lineBuffer[7], lineBuffer[8]);

	switch (type)
	{
		case DATARECORD:				// don't know how to cope with anything but 16-bit addresses for now
			byteIdx = 9;				// the ninth character is the high nibble of the first data byte
			byteCount = atoh(lineBuffer[1], lineBuffer[2]);
			byteAddress =
				atoh(lineBuffer[3], lineBuffer[4]) * 256 +
				atoh(lineBuffer[5], lineBuffer[6]) + extendedLinearAddress;
			break;

		case ENDRECORD:
			break;

		case SEGADDRESS:
			extendedLinearAddress =
				(atoh(lineBuffer[9], lineBuffer[10]) * 256 +
				atoh(lineBuffer[11], lineBuffer[12])) << 4;
			break;

		case EXTADDRESS:
			extendedLinearAddress =
				(atoh(lineBuffer[9], lineBuffer[10]) * 256 +
				atoh(lineBuffer[11], lineBuffer[12])) << 16;
			break;

		default:
			fprintf(stderr, "GetIntelRecord: Unknown code %d: %s\n", type, lineBuffer);
			done = true;
			break;
	}

	len = atoh(lineBuffer[1], lineBuffer[2]) * 2;
	len += 8;		// add count, address, and type overhead
	chksum = 0;

	for (i=0; i<len; i += 2)
		chksum += atoh(lineBuffer[i + 1], lineBuffer[i + 2]);

	chksum = ~chksum;
	chksum++;
	chksum &= 0xff;

	if (chksum != atoh(lineBuffer[len + 1], lineBuffer[len + 2]))
	{
		fprintf(stderr, "GetIntelRecord: Checksum error 0x%02x: %s\n", chksum, lineBuffer);
		done = true;
	}

	return(!done);
}

//-----------------------------------------------------------------------------
// interpret the line as a motorola s-record
// DEBUG should verify the checksum here
//  Motorola S record files not used with PIC
static bool GetMotRecord(FILE *theFile)
{
	bool	done;
		
	done = false;

	switch (lineBuffer[1])
	{
		case '0':				// comment record, don't fail but show that it didn't give us any new bytes
			byteCount = atoh(lineBuffer[2], lineBuffer[3]) - 3;	// don't count the address or checksum
			byteAddress = 0;
			byteIdx = 8;
			fprintf(stderr,"comment: ");

			while (byteCount)
			{
				fputc(atoh(lineBuffer[byteIdx], lineBuffer[byteIdx + 1]), stderr);	// display the comment
				byteIdx += 2;
				byteCount--;
			}

			fputc('\n', stderr);
			break;

		case '1':								// 16 bit address
			byteCount = atoh(lineBuffer[2], lineBuffer[3]) - 3;	// don't count the address or checksum
			byteAddress = (atoh(lineBuffer[4], lineBuffer[5]) << 8) +
								atoh(lineBuffer[6], lineBuffer[7]);
			byteIdx = 8;
			break;

		case '2':								// 24 bit address
			byteCount = atoh(lineBuffer[2], lineBuffer[3]) - 4;			// don't count the address or checksum
			byteAddress = (atoh(lineBuffer[4], lineBuffer[5]) << 16) +
								(atoh(lineBuffer[6], lineBuffer[7]) << 8) +
								atoh(lineBuffer[8], lineBuffer[9]);
			byteIdx = 10;
			break;

		case '3':								// 32 bit address
			byteCount = atoh(lineBuffer[2], lineBuffer[3]) - 5;			// don't count the address or checksum
			byteAddress = (atoh(lineBuffer[4], lineBuffer[5]) << 24) +
							(atoh(lineBuffer[6], lineBuffer[7]) << 16) +
							(atoh(lineBuffer[8], lineBuffer[9]) << 8) +
							atoh(lineBuffer[10], lineBuffer[11]);
			byteIdx = 12;
			break;

		case '9':								// end record
		case '7':								// also end record?
			break;

		default:
			done = true;
			break;
	}

	return(!done);
}

//-----------------------------------------------------------------------------
// read the next record, return true if read, false if end of file (or end record)
static bool GetRecord(FILE *theFile)
{
	bool	done, overflow, redo;
		
	done = false;
	overflow = false;		// set true when oversized line is read
	redo = true;			// for skipping comment lines

	while (redo && !done)
	{
		if (GetLine(theFile, lineBuffer, MAX_LINE_LEN, &done, &overflow))
		{
			if (overflow)
				fprintf(stderr, "Line too long, truncation occurred\n");

			if (!done)
			{
				if (lineBuffer[0] == INTEL_CHAR)
				{
					done = !GetIntelRecord(theFile);
					redo = false;
				}
				else if (lineBuffer[0] == MOT_CHAR)
				{
					done = !GetMotRecord(theFile);
					redo = false;
				}
				else if (lineBuffer[0] == COMMENT_CHAR || lineBuffer[0] == SPACE_CHAR
					|| lineBuffer[0] == NEWLINE_CHAR || lineBuffer[0] == NULL_CHAR)
				{
					byteCount = 0;
					redo = true;
				}
				else
				{
					fprintf(stderr, "Unrecognized record format: '%c' 0x%02x\n", lineBuffer[0], lineBuffer[0]);
					done = true;
					redo = false;
				}
			}
		}
	}

	return(!done);					// return true if okay
}

//-----------------------------------------------------------------------------
// read the next byte, report its address and value
// read next record as needed
// return true if read okay, false if end of file or error
bool GetNextByte(FILE *theFile, unsigned int *address, unsigned char *data)
{
	bool	fail;
		
	fail = false;

	while (!byteCount && !fail)			// do as often as necessary to skip comment records
		fail = !GetRecord(theFile);		// read in another record, if possible

	if (!fail)
	{
		*data = atoh(lineBuffer[byteIdx], lineBuffer[byteIdx + 1]);
		*address = byteAddress;
		byteIdx += 2;
		byteCount--;
		byteAddress++;
	}

	return(!fail);
}

//-----------------------------------------------------------------------------
// get ready to parse records
void InitParse()
{
	byteCount = 0;
	byteIdx = 0;
	byteAddress = 0;
	extendedLinearAddress = 0;
}


