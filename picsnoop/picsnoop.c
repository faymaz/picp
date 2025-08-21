//-----------------------------------------------------------------------------
//
//	PICSTART communications snooper
//
//-----------------------------------------------------------------------------
//
//	Copyright (C) 2002 Cosmodog, Ltd.
//	Copyright (C) 2005 Jeffery L. Post
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
//  Please see README.TXT for information on using this program
//
//-----------------------------------------------------------------------------
// thanks to Todd Squires for the orignal terminal interface
//-----------------------------------------------------------------------------
//
// Modified to add extra PIC_DEFINITION data for 18xxx support - JLP
// Additional definitions - JLP
// Aug 24, 2005 - Modified to generate entry for picdevrc file.
//

static const char * const version = "0.3";

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include	<ctype.h>

#ifdef WIN32
#include	<windows.h>
#define	false	FALSE
#define	true	TRUE
#else
#include <termio.h>
#endif

#define	byte		unsigned char
#define	word		unsigned short int

#include "serial.h"

#define	DEF_SIZE		44
#define	DEFX_SIZE	32

#define	LONG_FORMAT		0		// output picdevrc format with comments
#define	SHORT_FORMAT	1		// output picdevrc format without comments
#define	OLD_FORMAT		2		// output old picdev.c format

// Apparently Windoze doesn't know what errno is.
// Somehow that seems typical of the crap from Redmond.

#ifdef WIN32
extern int	_errno;
#define	errno	_errno
#else
extern int	errno;
#endif

static char *deviceName;
static int	theDevice;
static char	*picName;
static int	formatflag;
static char	*programName;
static bool	done;

//-----------------------------------------------------------------------------
// process signals (any signal will cause us to exit)
static void SigHandler(int sig)
{
	done = true;
}

//-----------------------------------------------------------------------------
// dump the data from the port in hex
void Dump(void)
{
	byte	theBuffer[10];
	int	numRead, numShown;
	
	numShown = 0;

	while (!done)
	{
		numRead = read(theDevice, theBuffer, 1);		// try to read a character

		if (numRead >= 1)
		{
			fprintf(stdout, "%02x ", theBuffer[0]);
			fflush(stdout);					// make sure it dumps it out right away
			numShown++;

			if (numShown == 16)
			{
				numShown = 0;
				fputc('\n', stdout);
			}
		}

		if (numRead < 0)
			fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
	}

	fputc('\n', stdout);
}

void outputOldFormat(byte *theBuffer)
{
	fprintf(stdout, "//-----------------------------------------------------------\n");
	fprintf(stdout, "const static unsigned char def_PICxxxxx[] =\n");
	fprintf(stdout, "{\n");
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// size of program space\n", theBuffer[0], theBuffer[1]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// width of address word\n", theBuffer[2], theBuffer[3]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// width of data word\n", theBuffer[4], theBuffer[5]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// width of ID\n", theBuffer[6], theBuffer[7]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// ID mask\n", theBuffer[8], theBuffer[9]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// width of configuration word\n", theBuffer[10], theBuffer[11]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// configuration word mask\n", theBuffer[12], theBuffer[13]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// EEPROM data width\n", theBuffer[14], theBuffer[15]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// EEPROM data mask\n", theBuffer[16], theBuffer[17]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// Calibration width\n", theBuffer[18], theBuffer[19]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// Calibration mask\n", theBuffer[20], theBuffer[21]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// ??\n", theBuffer[22], theBuffer[23]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// ??\n", theBuffer[24], theBuffer[25]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// address of ID locations\n", theBuffer[26], theBuffer[27]);
	fprintf(stdout, "\t0x%02x,\t\t\t// size of ID locations\n", theBuffer[28]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// address of configuration bits\n", theBuffer[29], theBuffer[30]);
	fprintf(stdout, "\t0x%02x,\t\t\t// size of configuration register\n", theBuffer[31]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// address of data space\n", theBuffer[32], theBuffer[33]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// size of data space\n", theBuffer[34], theBuffer[35]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// address of internal clock calibration value\n", theBuffer[36], theBuffer[37]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t// size of clock calibration space\n", theBuffer[38], theBuffer[39]);
	fprintf(stdout, "\t0x%02x,\t\t\t// additional programming pulses for C devices\n", theBuffer[40]);
	fprintf(stdout, "\t0x%02x,\t\t\t// main programming pulses for C devices\n", theBuffer[41]);
	fprintf(stdout, "\t0x%02x, 0x%02x,\t\t//  ZIF configuration ??\n", theBuffer[42], theBuffer[43]);
	fprintf(stdout, "};\n\n");

	fprintf(stdout, "const static unsigned char defx_PICxxxxx[] =\n");
	fprintf(stdout, "{\n");
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[46], theBuffer[47], theBuffer[48], theBuffer[49]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[50], theBuffer[51], theBuffer[52], theBuffer[53]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[54], theBuffer[55], theBuffer[56], theBuffer[57]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[58], theBuffer[59], theBuffer[60], theBuffer[61]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[62], theBuffer[63], theBuffer[64], theBuffer[65]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[66], theBuffer[67], theBuffer[68], theBuffer[69]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[70], theBuffer[71], theBuffer[72], theBuffer[73]);
	fprintf(stdout, "\t0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", theBuffer[74], theBuffer[75], theBuffer[76], theBuffer[77]);
	fprintf(stdout, "};\n\n");

	fprintf(stdout, "const static PIC_DEFINITION PICxxxxx =\n");
	fprintf(stdout, "{\n");
	fprintf(stdout, "\t\"xxxxx\",\t\t// device name\n");
	fprintf(stdout, "\tdef_PICxxxxx,\t\t// definition\n");
	fprintf(stdout, "\tdefx_PICxxxxx,\t\t// extended definition\n");
	fprintf(stdout, "\t0,\t\t\t// config word: code protect bit mask\n");
	fprintf(stdout, "\t0,\t\t\t// config word: watchdog bit mask\n");

	fprintf(stdout, "\n\t0,\t\t\t// Word alignment for writing to this device\n");
	fprintf(stdout, "\t0,\t\t\t// Configuration memory start address\n");
	fprintf(stdout, "\t0, 0,\t\t\t// ID Locations addr\n");
	fprintf(stdout, "\t0,\t\t\t// Data eeprom address\n");

	fprintf(stdout, "\t0,\t\t\t// number of words in cfg bits with factory set bits\n");
	fprintf(stdout, "\t{0, 0, 0, 0, 0, 0, 0, 0},\t// fixed bits mask\n");
		// we assume that all programmers support the new chip (fat chance)
	fprintf(stdout, "\t(P_PICSTART | P_WARP13 | P_JUPIC),\t// bit map of supporting programmers\n");

	fprintf(stdout, "};\n\n\n");
	fflush(stdout);					// make sure it dumps it out right away
}

void outputShortFormat(byte *theBuffer)
{
//	int	i;

// The following information is not included in the data sent by MPLAB

	fprintf(stdout, "\n[%s]\n", picName);
	fprintf(stdout, "\t0 0 0 0 0 0 0 0\n");
	fprintf(stdout, "\t0 0 0 0 0 0 0 0\n");
	fprintf(stdout, "PICSTART WARP13 JUPIC\n");

// This is the data sent by the 0x81 command

	fprintf(stdout, "\n[%s:def]\n", picName);

// This is the data sent by the 0x82 command

	// incomplete

	fflush(stdout);
}

void outputLongFormat(byte *theBuffer)
{
	// incomplete
}

//-----------------------------------------------------------------------------
// capture profiles, display them in a usable format
void CaptureProfiles(void)
{
	const char	preamble[] = {0x88,0x8d,0x80,0x81};
	byte			theBuffer[256];
	int			numRead, idx;
		
	while (!done)
	{
		// first try to find the preamble (initial communications from MPLAB to the PICSTART)
		for (idx=0; idx < (int) sizeof(preamble) && !done; idx++)
		{
			do
			{
				numRead = read(theDevice, theBuffer, 1);			// read one character
			}while (numRead == 0 && !done);
			
			if (numRead == 1 && *theBuffer != preamble[idx])
				idx = 0;										// mismatch, start over

			if (numRead < 0)
				fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		}

		idx = 0;

		while (idx < DEF_SIZE + DEFX_SIZE + 3 && !done)
		{
			// the extra three bytes are the two checksums and the command in the middle (before the second block)
			numRead = read(theDevice, &theBuffer[idx], DEF_SIZE + DEFX_SIZE + 3 - idx);
			// try to read a total of DEF_SIZE + DEFX_SIZE + 3 bytes into the buffer

			if (numRead > 0)
				idx += numRead;

			if (numRead < 0)
				fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		}

		if (!done)
		{
			switch (formatflag)
			{
				case OLD_FORMAT:
					outputOldFormat(theBuffer);
					break;

				case SHORT_FORMAT:
					outputShortFormat(theBuffer);
					break;

				case LONG_FORMAT:
					outputLongFormat(theBuffer);
					break;
			}
		}
	}
}

//-----------------------------------------------------------------------------
// Once the device is opened and locked, this sets
// up the port, and makes sure the handshake looks good.
static bool InitDevice(int theDevice, int baudRate, byte dataBits, byte stopBits, byte parity)
{
	bool	CTS, DCD;

	if (ConfigureDevice(theDevice, baudRate, dataBits, stopBits, parity, false))	// set up the device
	{
		if (ConfigureFlowControl(theDevice, false))		// no flow control at the moment (raise RTS)
		{
			SetDTR(theDevice, true);							// raise DTR
			GetDeviceStatus(theDevice, &CTS, &DCD);		// see if CTS is true

			if (!CTS)
				fprintf(stderr, "Flow control is disabled\n");
			else
				ConfigureFlowControl(theDevice, true);	// looks ok to use flow control, so allow it

			FlushBytes(theDevice);						// get rid of any pending data
			return(true);
		}
		else
			fprintf(stderr, "Could not configure flow control\n");
	}
	else
		fprintf(stderr, "Could not configure device parameters\n");

	return(false);
}

//-----------------------------------------------------------------------------
// tell the user how to use this program
void Usage()
{
	fprintf(stderr, "Usage: %s ttyname [device] [-s | -o]\n", programName);
	fprintf(stderr, "  If device is specified then data will be formatted for use in picdevrc,\n");
	fprintf(stderr, "  else data from port will be dumped in hex.");
	fprintf(stderr, "  If -s option, generate picdevrc entry in short format.\n");
	fprintf(stderr, "  If -o option, generate old style picdev.c entry.\n\n");
}

void parseInput(int argc, char *argv[])
{
	int	i;
	char	*cptr;

	for (i=1; i<argc; i++)
	{
		cptr = argv[i];

		switch (toupper(*cptr))
		{
			case '-':		// option
				cptr ++;

				if (toupper(*cptr == 'S'))		// short output flag
					formatflag = SHORT_FORMAT;
				else if (toupper(*cptr == 'O'))
					formatflag = OLD_FORMAT;

				break;

#ifdef WIN32
			case 'C':		// comm port
#else
			case '/':		// serial port
#endif
				deviceName = argv[i];
				break;

			default:			// assume PIC device
				picName = argv[i];
				break;
		}
	}
}

//-----------------------------------------------------------------------------
// open the device, and begin terminal operations on it
int main(int argc,char *argv[])
{
	int	baudRate;
	byte	dataBits, stopBits, parity;

	done = false;
	formatflag = LONG_FORMAT;
	signal(SIGINT, SigHandler);			// set up a signal handler

	programName = argv[0];
	fprintf(stderr, "\n%s version 0.1 (c) 2002 Cosmodog, Ltd.\n", programName);
	fprintf(stderr, "%s version %s (c) 2005 Jeff Post.\n\n", programName, version);
	parseInput(argc, argv);
	
	if (argc >= 2)
	{
		if (OpenDevice(deviceName, &theDevice))				// open the device
		{
			baudRate = 19200;
			dataBits = 8;
			stopBits = 1;
			parity = 0;

			if (InitDevice(theDevice, baudRate, dataBits, stopBits, parity))
			{
				GetDeviceConfiguration(theDevice, &baudRate, &dataBits, &stopBits, &parity);
				fprintf(stderr, "%d,%d,%d,%c\n", baudRate, dataBits, stopBits, (parity == 0) ? 'N' : (parity == 1) ? 'O' : 'E');
				fprintf(stderr, "Type <ctrl-C> to exit\n");

				if (argc >= 3)
					CaptureProfiles();
				else
					Dump();

				fprintf(stderr, "exiting...\n");
			}
			else
				fprintf(stderr, "Failed to set up the serial port\n");

			CloseDevice(theDevice);
		}
		else
			fprintf(stderr, "Failed to open device '%s'\n", argv[1]);
	}
	else
		Usage();

	return(0);
}

// end
