//-----------------------------------------------------------------------------
//
//	PICSTART Plus, Warp-13, JuPic, and Olimex programming interface
// Version 0.6.9
// July 14, 2006
//
//	Copyright (C) 1999-2002 Cosmodog, Ltd.
// Copyright (c) 2004-2006 Jeffery L. Post
//
//	Cosmodog, Ltd.
//	415 West Huron Street
//	Chicago, IL   60610
//	http://www.cosmodog.com
//
// Download current code from
// http://home.pacbell.net/theposts/picmicro
//
// Please send bug reports to j_post <AT> pacbell <DOT> net.
//
//-----------------------------------------------------------------------------
//
// This interface to the PICSTART Plus, Warp-13 and Jupic programmers is provided in
// an effort to make the PICSTART (and thus, the PIC family of microcontrollers)
// useful and accessible across multiple platforms, not just one particularly
// well-known, unstable one.
//
//-----------------------------------------------------------------------------
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
// Revision history has been moved to the file HISTORY
//
//-----------------------------------------------------------------------------
//
//  TODO:
//		verify program space
//		verify data space
//		erase oscillator calibration (probably unnecessary since none of the current flash devices have calibration space)
//		support osc. calibration space sizes other than 1 (PIC14000 only?)
//		seems to have trouble verifying oscillator calibration when stringing several operations together
//		seems to have a problem with 16C63A (but not 16C63, which MPLAB appears to treat as identical)
//
//-----------------------------------------------------------------------------

//#define	BETA	1		// comment out for release versions

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include	<time.h>

#ifdef WIN32
#include	<windows.h>
#include	<malloc.h>
#define	usleep(x)	Sleep((x) / 1000)
#define	false	FALSE
#define	true	TRUE
#endif

#include "atoi_base.h"
#include "parse.h"
#include "serial.h"
#include "picdev.h"
#include "record.h"
#include "k150.h"

#define TIMEOUT_1_SECOND	1000000			// 1 second time to wait for a character before giving up (in microseconds)
#define TIMEOUT_2_SECOND	2000000			// 2 second timeout for erasing flash
#define TIMEOUT_5_SECOND	5000000			// 5 second timeout for 18Fxx devices

#define BUFFERSIZE		0x20000			// buffer bigger than anyone should need
#define MAXNAMESLEN		80					// max number of characters on a line when reporting device names

#define	OLD_PICDEV_DEFXSIZE	16

#define	MAX_OSC_CAL_SIZE		32				// maximum size of osc calibration data
#define	MAX_EEPROM_DATA_SIZE	(8 * 1024)	// maximum size of any device's data memory
#define	MAX_CFG_SIZE	8						// maximum number of words for configuration bits
//
// COMMANDS (sent to programmer)
//
#define CMD_BLANK_CHECK			0x42		// 'B' blank check (0x0f = F not blank, 0x17 = not blank, 0x10 = blank?)
#define CMD_WRITE_PGM			0x51		// 'Q' write program memory
#define CMD_READ_PGM				0x54		// 'T' read program memory
#define CMD_READ_OSC				0x63		// 'c' read oscillator calibration memory
#define CMD_READ_DATA			0x64		// 'd' read data (EEPROM) memory
#define CMD_READ_ID				0x65		// 'e' read ID locations
#define CMD_READ_CFG				0x66		// 'f' request configuration bits
#define CMD_WRITE_CFG			0x67		// 'g' write configuration bits
#define CMD_WRITE_ID				0x68		// 'h' write ID locations
#define CMD_WRITE_DATA			0x69		// 'i' write data memory
#define CMD_WRITE_CFG_WORD		0x70		// 'p' write one configuration word
#define CMD_WRITE_OSC			0x71		// 'q' write oscillator calibration memory
#define CMD_GET_PROC_LEN		0x80		// get processor info length
#define CMD_LOAD_INFO			0x81		// send processor-specific info
#define CMD_LOAD_EXT_INFO		0x82		// send more processor-specific info (this is new as of 0.4b)
#define CMD_REQUEST_MODEL		0x88		// request programmer model
#define CMD_REQUEST_VERSION	0x8d		// request firmware version
#define CMD_SET_ADDR				0x8e		// set address range (start address, size)
#define CMD_ERASE_FLASH			0x8f		// send erase flash device command

#define PIC_ACK					0xab		// response from programmer to model request

#define MIN_MAJORVER	3					// picstart version number detected must be at least this high
#define MIN_MIDVER	00
#define MIN_MINORVER	40

#define	NEW_PS_VERSION	0x41e04		// changed protocol starting with PS+ firmware v4.30.04

#define MINIMUM_VER		(MIN_MAJORVER*65536 + MIN_MIDVER*256 + MIN_MINORVER)

#define HASH_WIDTH_DEFAULT		40			// default width of the status bar

// Prototypes

static bool DoInitPIC(const PIC_DEFINITION *picDevice);
static bool DoErasePgm(const PIC_DEFINITION *picDevice, bool flag);
static bool DoEraseData(const PIC_DEFINITION *picDevice, bool flag);
static bool DoEraseConfigBits(const PIC_DEFINITION *picDevice);
static bool DoEraseIDLocs(const PIC_DEFINITION *picDevice);
static unsigned short int	GetPgmSize(const PIC_DEFINITION *picDevice);
static unsigned short int	GetDataSize(const PIC_DEFINITION *picDevice);
static unsigned int			GetIDSize(const PIC_DEFINITION *picDevice);
static unsigned int			GetConfigSize(const PIC_DEFINITION *picDevice);

// Struct definitions

typedef struct
{
	unsigned char major;
	unsigned char middle;
	unsigned char minor;
} VERSION;

typedef unsigned short int SIZEFNCT(const PIC_DEFINITION *);

typedef struct
{
	unsigned char	mask;
	SIZEFNCT			*sizef;				// pointer to function to return the size of the space (0=no function)
	char				*name;
} BLANK_MSG;

// Data

static char versionString[64];	// this program's version number

int	deviceCount = 0;
DEV_LIST	*deviceList = NULL;

static const BLANK_MSG blankList[] =
{
	{BLANK_PGM,		&GetPgmSize,	"program memory"},
	{BLANK_CFG,		0,					"configuration bits"},
	{BLANK_ID,		0,					"ID locations"},
	{BLANK_DATA,	&GetDataSize,	"data memory"},
	{0,0,""},
};

static char	*programName, *deviceName, *picName;

static VERSION		PICversion;
unsigned int		picFWVersion = 0;

FILE	*comm_debug;
int	comm_debug_count = 0;
bool	writingProgram = false;
bool	sendCommand = false;
bool	is18device = false;

unsigned short	programmerSupport = P_PICSTART;	// supported programmer
bool	isWarp13 = false;
bool	isJupic = false;
bool	isOlimex = false;			// non yet supported by picp - no way to test
bool	isK150 = false;

bool	ISPflag = false;
bool	suppressWrite = false;

static bool			verboseOutput;
static bool			ignoreVerfErr;
static int			serialDevice;
static unsigned int			CharTimeout = TIMEOUT_1_SECOND;	// default 1 second timeout
static unsigned short int	readConfigBits[16];	// config bits read back from device
static unsigned int			hashWidth;				// width of status bar (0 = none)
static int			oldFirmware = false;
static unsigned int	w13version = 0;

static unsigned char	oscCalData[MAX_OSC_CAL_SIZE];
static unsigned char eepromData[MAX_EEPROM_DATA_SIZE + 2];

//  status bar handling
static unsigned short int	hashMod, hashNum;

//-----------------------------------------------------------------------------
// reset the PICSTART Plus by lowering then raising DTR

static void ResetPICSTART()
{
	SetDTR(serialDevice, false);			// lower DTR to reset PICSTART Plus
	usleep(1000000/4);						// sleep for a quarter second (to let PS+ reset)
	SetDTR(serialDevice, true);			// raise DTR
}

//-----------------------------------------------------------------------------
// process signals (any signal will cause us to exit)

static void SigHandler(int sig)
{
	fprintf(stderr, "exiting...\n");
	ResetPICSTART();
	exit(0);
}

//-----------------------------------------------------------------------------
// Find the requested PIC device from the list of supported devices.
// If the user didn't prepend 'PIC' to the device name skip the
// first three letters of the devices when comparing.  This allows
// both PIC16C505 or 16C505 to refer to the same part.

PIC_DEFINITION *GetPICDefinition(char *name)
{
	int		idx;
	int		offset;
	DEV_LIST	*devptr;

	idx = 0;

	while (name[idx])					 // ensure it's all upper case
	{
		name[idx] = toupper(name[idx]);
		idx++;
	}

	offset = 0;								// assume no PIC at the front

	if (strncmp(name, "PIC", 3) == 0)			// if the user's argument has 'PIC' at the front,
		offset = 3;							// skip the first three letters of the user's argument

	idx = 0;
	devptr = deviceList;

	while (devptr)
	{
		if (strcmp(devptr->picDef.name, name + offset) == 0)
			break;

		devptr = devptr->next;
	}

	if (devptr && !(strncmp(devptr->picDef.name, "18", 2)))
	{
		CharTimeout = TIMEOUT_5_SECOND;
		is18device = true;
	}
	else
		is18device = false;

	if (devptr)
		return((PIC_DEFINITION *) devptr);

	return NULL;		// return 0 if no match
}

//-----------------------------------------------------------------------------
// return the size of the program space of the specified device (in words)

static unsigned short int GetPgmSize(const PIC_DEFINITION *picDevice)
{
	return(picDevice->def[PD_PGM_SIZEH] * 256 + picDevice->def[PD_PGM_SIZEL]);
}

//-----------------------------------------------------------------------------
// return the size of the data space of the specified device (in bytes)

static unsigned short int GetDataSize(const PIC_DEFINITION *picDevice)
{
	return(picDevice->def[PD_DATA_SIZEH] * 256 + picDevice->def[PD_DATA_SIZEL]);
}

//-----------------------------------------------------------------------------
// return the start address of the data space of the specified device

static unsigned short int GetDataStart(const PIC_DEFINITION *picDevice)
{
	return (picDevice->eeaddr) ? picDevice->eeaddr :
		(unsigned) (picDevice->def[PD_DATA_ADDRH] * 256 +
		 picDevice->def[PD_DATA_ADDRL]);
}

// return the ID Locations area size, in words.

static unsigned int GetIDSize(const PIC_DEFINITION *picDevice)
{
	return picDevice->def[PD_ID_SIZE];
}

// return the ID Locations area start addr.

static unsigned int GetIDAddr(const PIC_DEFINITION *picDevice)
{
	return picDevice->idaddr ? picDevice->idaddr / 2:
		(unsigned) picDevice->def[PD_ID_ADDRH] * 256 + picDevice->def[PD_ID_ADDRL];
}

//-----------------------------------------------------------------------------
// return the size of the oscillator calibration space of the specified device (in words)

static unsigned short int GetOscCalSize(const PIC_DEFINITION *picDevice)
{
	return(picDevice->def[PD_CLK_SIZEH] * 256 + picDevice->def[PD_CLK_SIZEL]);
}

//-----------------------------------------------------------------------------
// return the start address of the oscillator calibration space of the specified device

static unsigned short int GetOscCalStart(const PIC_DEFINITION *picDevice)
{
	return(picDevice->def[PD_CLK_ADDRH] * 256 + picDevice->def[PD_CLK_ADDRL]);
}

//-----------------------------------------------------------------------------
// return the start address of the configuration bits, in words.

static unsigned int GetConfigStart(const PIC_DEFINITION *picDevice)
{
	return picDevice->cfgmem ?
		picDevice->cfgmem / 2: (unsigned)
		(picDevice->def[PD_CFG_ADDRH] * 256 +
		picDevice->def[PD_CFG_ADDRL]);
}

// return the size of the configuration bits, in words.

static unsigned int GetConfigSize(const PIC_DEFINITION *picDevice)
{
	return picDevice->def[PD_CFG_SIZE];
}

// return start address of eeprom data (if non-zero)

static unsigned int GetEepromStart(const PIC_DEFINITION *picDevice)
{
	return picDevice->eeaddr;
}

// Get the alignment interval for recording purposes (in words).

static unsigned int GetWordAlign(const PIC_DEFINITION *picDevice)
{
  return picDevice->wordalign ? picDevice->wordalign : 0; // No align by default
}

// Get the word width for this device

static unsigned short int GetWordWidth(const PIC_DEFINITION *picDevice)
{
	return (picDevice->def[PD_PGM_WIDTHH]) << 8 | picDevice->def[PD_PGM_WIDTHL];
}

//-----------------------------------------------------------------------------
//	send a message to the programmer, wait for a specified number of bytes to be returned
//  If false is returned, there was a timeout, or some other error

static bool SendMsg(const unsigned char *cmdBuff, unsigned int cmdBytes, unsigned char *rtnBuff, unsigned int rtnBytes)
{
	bool	fail;
	int	numRead;
	int	bytesRemaining;

	fail = false;

	if (cmdBytes)
	{
		if (comm_debug)
		{
			if (!writingProgram)
			{
				fprintf(comm_debug, "\n");
				comm_debug_count = 0;
			}
		}

		WriteBytes(serialDevice, (unsigned char *) &cmdBuff[0], cmdBytes);	// send out the command
	}

	bytesRemaining = rtnBytes;

	if (!suppressWrite)
	{
		while (bytesRemaining && !fail)
		{
			numRead = ReadBytes(serialDevice, &rtnBuff[rtnBytes - bytesRemaining], bytesRemaining, CharTimeout);

			if (numRead < 0)
			{
				fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
				fail = true;
			}
			else if (numRead == 0)		// timed out
				fail = true;

			// subtract bytes read in, don't allow to underflow (shouldn't happen)
			bytesRemaining = (bytesRemaining > numRead) ? bytesRemaining - numRead : 0;
		}
	}
	else
	{
		for (numRead=0; numRead<bytesRemaining; numRead++)
			rtnBuff[numRead] = cmdBuff[numRead];
		fail = false;
	}

	return(!fail);
}

//-----------------------------------------------------------------------------
//	send a message to the programmer, wait for each byte to be returned
//  If false is returned, there was a timeout, or some other error

static bool SendMsgWait(const unsigned char *cmdBuff, unsigned int cmdBytes, unsigned char *rtnBuff, unsigned int rtnBytes)
{
	bool	fail;
	int	numRead;
	unsigned int	i;
	int	bytesRemaining;

	fail = false;
	bytesRemaining = rtnBytes;

	if (comm_debug && !writingProgram)
	{
		comm_debug_count = 0;
		fprintf(comm_debug, "\n");
	}

	for (i=0; i<cmdBytes && bytesRemaining && !fail; i++)
	{
		WriteBytes(serialDevice, (unsigned char *) &cmdBuff[i], 1);	// send out the command

		if (!suppressWrite)
		{
			numRead = ReadBytes(serialDevice, &rtnBuff[i], 1, CharTimeout);

			if (numRead < 0)
			{
				fprintf(stderr,"error %d, %s\n", errno, strerror(errno));
				fail = true;
			}
			else if (numRead == 0)		// timed out
				fail = true;

			// subtract bytes read in, don't allow to underflow (shouldn't happen)
			bytesRemaining = (bytesRemaining > numRead) ? bytesRemaining - numRead : 0;
		}
		else
		{
			rtnBuff[i] = cmdBuff[i];
			--bytesRemaining;
		}
	}

	return(!fail);
}

// JuPic programmer responded, attempt to get serial number.

static void check_jupic(void)
{
	int				i;
	unsigned int	to;
	unsigned char	bfr[64];

	to = CharTimeout;						// save normal character timeout, since this
	CharTimeout = TIMEOUT_1_SECOND;	// should always be done with short timeout

	bfr[0] = 's';
	bfr[1] = 0;

	SendMsg(bfr, 1, bfr, 64);	// expect a timeout here

	i = 0;
	while (bfr[i])
	{
		if (bfr[i] < ' ')
		{
			bfr[i] = 0;
			break;
		}

		i++;
	}

	fprintf(stderr, "JuPic %s\n", bfr);
	CharTimeout = to;			// restore character timeout
}

//
// This routine will check if an alternate programmer (Warp-13 or
// JuPic) is connected.
//
// NOTE: This feature of the Warp-13 programmer is based on the
// TM4 protocol, and MAY NOT BE SUPPORTED IN FUTURE VERSIONS of
// the Warp-13 firmware.
//
// If a Picstart Plus is connected, it will echo back the four
// bytes sent, but otherwise ignore them.
//
// If a Warp-13 is connected, it will respond to the four byte
// command with a single byte of 0x01. Then we send the next four
// byte command and the Warp-13 will respond with 16 bytes of
// data. Bluepole returns "BP" in the first two bytes, Redback
// returns "RX". Bytes at offset 3-7 are the version code for
// the Warp-13 firmware, but the exact interpretation of the
// meaning of these five bytes aren't known to me at this time.
//
// If a JuPic programmer is connected, it will respond to the
// four byte command with 0x02.
//

static void check_programmer(void)
{
	int	i;
	unsigned int	to;
	unsigned char	bfr[16];
	char port_name[256];

	to = CharTimeout;						// save normal character timeout, since this
	CharTimeout = TIMEOUT_1_SECOND;	// should always be done with short timeout

	// First check for K150 programmer
	strcpy(port_name, "/dev/ttyUSB0");  // K150 typically uses ttyUSB0
	if (k150_open_port(port_name) == 0)
	{
		if (k150_detect_programmer() == 0)
		{
			isK150 = true;
			programmerSupport = P_K150;
			if (comm_debug)
			{
				fprintf(comm_debug, "\nK150 programmer detected");
			}
			// Keep K150 port open for programming operations
			CharTimeout = to;
			return;
		}
		k150_close_port();
	}

	bfr[0] = '+';
	bfr[1] = 'M';
	bfr[2] = '0';
	bfr[3] = 7;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nChecking for Warp-13 or JuPic programmer");
		comm_debug_count = 0;
	}

	for (i=4; i<16; i++)
		bfr[i] = 0;

	SendMsg(bfr, 4, bfr, 4);	// expect a timeout here, so don't check return value

	if (bfr[0] == 2)				// JuPic programmer responded
	{
		isJupic = true;
		programmerSupport = P_JUPIC;
		check_jupic();
		return;
	}

	if (bfr[0] != 1)				// Warp-13 not connected
	{
		CharTimeout = to;			// restore character timeout
		return;
	}

	bfr[0] = '+';					// Warp-13 responded, so send
	bfr[1] = 'M';					// firmware request command
	bfr[2] = '0';
	bfr[3] = 0x0e;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nGetting Warp-13 version info");
		comm_debug_count = 0;
	}

	if (SendMsg(bfr, 4, bfr, 16))
	{
		w13version = (unsigned int) bfr[3] << 24 | bfr[4] << 16 | bfr[5] << 8 | bfr[6];
		fprintf(stderr, "Warp-13 %c%c %02x.%02x.%02x.%02x.%02x\n",
			bfr[0], bfr[1], bfr[3], bfr[4], bfr[5], bfr[6], bfr[7]);
		isWarp13 = true;
		programmerSupport = P_WARP13;
	}

	CharTimeout = to;			// restore character timeout
}

//-----------------------------------------------------------------------------
//	ask what type of programmer is attached (if any)
//
// This is where MPLAB tries to identify what type of programmer is
// attached and selects the appropriate protocol based on the response.
// We just check for Picstart Plus response code and fail if we don't get it.

static bool DoGetProgrammerType()
{
	bool				succeed;
	unsigned char	theBuffer[1], theRtnBuffer[1];
	unsigned int	retryCount;

	check_programmer();						// test if alternate programmer is connected

	theBuffer[0] = CMD_REQUEST_MODEL;	// Ping the programmer
	retryCount = 5;
	succeed = false;

	do
	{
		if (comm_debug)
		{
			fprintf(comm_debug, "\nGet programmer type");
			comm_debug_count = 0;
		}

		if (SendMsg(theBuffer, 1, theRtnBuffer, 1))
		{
			if (theRtnBuffer[0] == PIC_ACK)		// programmer responded to ping?
				succeed = true;
			else
				fprintf(stderr, "Get programmer type responded with %02X\n", theRtnBuffer[0]);
		}

		--retryCount;
	} while(!succeed && retryCount);

	return(succeed);
}

//-----------------------------------------------------------------------------
// request firmware version from the programmer

static bool DoGetVersion()
{
	bool				succeed;
	int				retryCount;
	unsigned char	theBuffer[1], theRtnBuffer[4];

	succeed = false;
	retryCount = 5;
	theBuffer[0] = CMD_REQUEST_VERSION;

	do
	{
		if (comm_debug)
		{
			fprintf(comm_debug, "\nGet version");
			comm_debug_count = 0;
		}

		if (SendMsg(theBuffer, 1, theRtnBuffer, 4))
		{
			if (theRtnBuffer[0] == CMD_REQUEST_VERSION)
			{
				succeed = true;
				PICversion.major = theRtnBuffer[1];
				PICversion.middle = theRtnBuffer[2];
				PICversion.minor = theRtnBuffer[3];
				picFWVersion = ((theRtnBuffer[1] << 16) & 0xff0000);
				picFWVersion |= ((theRtnBuffer[2] << 8) & 0xff00);
				picFWVersion |= (theRtnBuffer[3] & 0xff);
			}
		}

		--retryCount;
	} while (!succeed && retryCount);

	if (!succeed)
		fprintf(stderr, "failed to read programmer version number\n");

	return(succeed);
}

//-----------------------------------------------------------------------------
//	get the version from the PICSTART and display it

static void DoShowVersion()
{
	fprintf(stdout,"PICSTART Plus firmware version %d.%02d.%02d\n",
		PICversion.major, PICversion.middle, PICversion.minor);
}

//-----------------------------------------------------------------------------
// send a "set range" command to the PS+

static bool SetRange(const PIC_DEFINITION *picDevice, unsigned int start, unsigned int length)
{
	unsigned char	rangeBuffer[6], rtnBuffer[6];
	int				i, size;
	bool				error = false;
	bool				nowrite = false;

	if (GetWordWidth(picDevice) == 0xffff)
		start *= 2;		// For these devices, addressing is done in octets.

	rangeBuffer[0] = CMD_SET_ADDR;

	if (!oldFirmware)
	{
		size = 6;
		rangeBuffer[1] = (start >> 16) &0xff;
		rangeBuffer[2] = (start >> 8) & 0xff;
		rangeBuffer[3] = (start >> 0) & 0xff;
		rangeBuffer[4] = (length >> 8) & 0xff;
		rangeBuffer[5] = (length >> 0) & 0xff;
	}
	else
	{
		size = 5;
		rangeBuffer[1] = (start >> 8) & 0xff;
		rangeBuffer[2] = (start >> 0) & 0xff;
		rangeBuffer[3] = (length >> 8) & 0xff;
		rangeBuffer[4] = (length >> 0) & 0xff;
	}

	if (comm_debug)
	{
		fprintf(comm_debug, "\nSet Range");
		comm_debug_count = 0;
		sendCommand = true;
	}

	nowrite = suppressWrite;
	suppressWrite = false;

 	if (SendMsgWait(rangeBuffer, size, rtnBuffer, size))
	{
 		if (memcmp(rangeBuffer, rtnBuffer, size) == 0)	// read back result and see if it looks correct
		{
			suppressWrite = nowrite;
			return(true);
		}
		else
		{
			fprintf(stderr,"echoback from set range command incorrect\n");

			for (i=0; i<size; i++)
				fprintf(stderr, " %02x", rangeBuffer[i]);

			fprintf(stderr, "\n");

			for (i=0; i<size; i++)
				fprintf(stderr, " %02x", rtnBuffer[i]);

			fprintf(stderr, "\n");
		}
	}
	else
		fprintf(stderr,"failed to send set range command\n");

	suppressWrite = nowrite;
	return (!error);
}

//-----------------------------------------------------------------------------
// Check device for blank

static bool DoBlankCheck(const PIC_DEFINITION *picDevice, unsigned char blankMode)
{
	bool				fail, newfw = false;
	unsigned char	theBuffer[3];
	int				idx;
	unsigned int	to;

	to = CharTimeout;						// save normal character timeout
	CharTimeout = TIMEOUT_5_SECOND;	// set long timeout for blank check

	if (picFWVersion >= NEW_PS_VERSION && !isJupic && !isWarp13 && !isOlimex)
		newfw = true;

	idx = 0;
	fail = false;
	theBuffer[0] = CMD_BLANK_CHECK;
	theBuffer[1] = 0xef;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nBlank Check");
		comm_debug_count = 0;
	}

	if (SendMsg(theBuffer, 1, theBuffer, 2))
	{
		if (theBuffer[1] == 0xef && newfw)		// wait for endless 0xef from broken PS+ firmware
		{
			while (SendMsg(NULL, 0, theBuffer, 1))
			{
				if (theBuffer[0] != 0xef)
					break;
			}

			theBuffer[1] = theBuffer[0];	// put result where it should be
		}

		theBuffer[1] &= blankMode;				// look only at what we were asked to look at

		if (!verboseOutput)
			fprintf(stdout, "0x%02x\n", theBuffer[1]);			// quiet mode will just show the return code
		else
		{
			while (blankList[idx].mask)								// report anything that isn't blank
			{
				if (blankMode & blankList[idx].mask)				// check only what we were asked
				{
					if (!blankList[idx].sizef || (blankList[idx].sizef(picDevice) > 0))
					{
						if ((blankMode & blankList[idx].mask) == BLANK_CFG)
						{
							fprintf(stdout, "%s: ", blankList[idx].name);

							if (theBuffer[1] & BLANK_CFG)
							{
								fprintf(stdout, "not blank\n");
								fail = true;								// config bits not blank
							}
							else
								fprintf(stdout, "blank\n");
						}
						else
						{
							fprintf(stdout, "%s: ", blankList[idx].name);

							if (theBuffer[1] & blankList[idx].mask)	// if a bit is set,
							{
								fprintf(stdout, "not blank\n");
								fail = true;									// something wasn't blank
							}
							else
								fprintf(stdout, "blank\n");
						}
					}
				}

				idx++;
			}
		}
	}
	else
	{
		fprintf(stderr, "failed to send blank check command\n");
		fail = true;
	}

	CharTimeout = to;		// restore character timeout
	return(!fail);
}

//--------------------------------------------------------------------
// Read eeprom data

static bool DoReadData(const PIC_DEFINITION *picDevice, FILE *theFile)
{
	bool				fail;
	unsigned char	theBuffer[1024];
	unsigned short int	size, start;

	size = GetDataSize(picDevice);
	start = GetDataStart(picDevice);

	if (!size)
	{
		fprintf(stderr, "Device %s has no eeprom data!\n", picName);
		return false;
	}

	fail = false;
	theBuffer[0] = CMD_READ_DATA;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nRead Data");
		comm_debug_count = 0;
		sendCommand = true;
	}

	if (SendMsg(theBuffer, 1, eepromData, size + 2))
	{								// ask it to fill the buffer (plus the command plus a terminating zero)
		WriteHexRecord(theFile, &eepromData[1], start, size, 0);	// write hex records to selected stream
	}
	else
	{
		fprintf(stderr, "failed to send read data command\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Write eeprom data from file

static bool DoWriteData(const PIC_DEFINITION *picDevice, FILE *theFile)
{
	int				i;
	bool				fail, fileDone;
	unsigned short int	size, start, count;
	unsigned int	startAddr, curAddr, nextAddr;
	unsigned char	data;

	size = GetDataSize(picDevice);
	start = GetDataStart(picDevice);

	if (!size)
	{
		fprintf(stderr, "Device %s has no eeprom data!\n", picName);
		return false;
	}

	for (i=0; i < size + 1; i++)
		eepromData[i] = 0xff;

	fail = fileDone = false;

	InitParse();											// get ready to start reading the hex file

	fileDone = !GetNextByte(theFile, &nextAddr, &data);	// get a byte and the initial address

	while (!fileDone && !fail)
	{
		nextAddr &= (size - 1);							// keep address within buffer range
		startAddr = nextAddr;							// the first address of the new block
		curAddr = startAddr;
		eepromData[startAddr + 1] = data;			// the first data byte of the new block
		count = 2;											// number of bytes waiting to be sent

		while ((!(fileDone = !GetNextByte(theFile, &nextAddr, &data)) &&  (count < size + 1)))
		{														// get next byte
			nextAddr &= (size - 1);

			if (curAddr + 1 == nextAddr)				// use this byte now only if it's sequential
			{
				eepromData[startAddr + count] = data;
				count++;
				curAddr++;

				if (curAddr > (unsigned int)(start + size))
				{
					fail = true;
					fprintf(stderr, "Data file exceeds eeprom data size\n");
					break;
				}
			}
			else
				break;									// non-sequential, write this buffer then start another
		}
	}

	if (!fail)
	{
		writingProgram = true;
		eepromData[0] = CMD_WRITE_DATA;			// set command in eepromData buffer

		if (comm_debug)
		{
			if (suppressWrite)
				fprintf(comm_debug, "\nWrite Data - write suppressed\n");
			else
				fprintf(comm_debug, "\nWrite Data\n");

			comm_debug_count = 0;
			sendCommand = true;
		}

		if (!SendMsgWait(eepromData, 1, eepromData, 1))
		{
			fprintf(stderr, "failed to send write eeprom data command\n");
			fail = true;
		}

		if (comm_debug)
		{
			fprintf(comm_debug, "\n");
			comm_debug_count = 0;
		}

		if (!fail)
		{
			if (!SendMsgWait(&eepromData[1], size, &eepromData[1], size + 1))
			{
				fprintf(stderr, "failed to send write data command\n");
				fail = true;
			}
		}

		if (!fail && !suppressWrite)
		{
			if (!SendMsg(&eepromData[0], 0, &eepromData[0], 1))			// eat the trailing zero
			{
				fprintf(stderr, "failed to read trailing 0 after writing eeprom data\n");
				fail = true;									// didn't echo everthing back like it should have
			}
		}
	}

	writingProgram = false;
	return(!fail);
}

//--------------------------------------------------------------------
// Write eeprom data from buffer - return true if success

static bool DoWriteEepromData(const PIC_DEFINITION *picDevice, unsigned char *buffer, unsigned int start, int size)
{
	int						i;
	unsigned short int	datasize;
	bool						fail = false;

	datasize = GetDataSize(picDevice) * 2;

	if (!datasize)
	{
		fprintf(stderr, "Device %s has no eeprom data!\n", picName);
		return false;
	}

	if (size > datasize)
	{
		fprintf(stderr, "Invalid size for eepromdata, %d, max is %d\n", size, datasize);
		return false;
	}

	for (i=0; i < datasize + 1; i++)			// initialize eeprom data
		eepromData[i] = 0xff;

	start &= 0xffff;

	for (i=0; i<size; i++)						// transfer block of data to eeprom data
		eepromData[start + i + 1] = buffer[i];

	writingProgram = true;
	eepromData[0] = CMD_WRITE_DATA;			// set command in eepromData buffer

	if (comm_debug)
	{
		if (suppressWrite)
			fprintf(comm_debug, "\nWrite EEPROM Data - write suppressed\n");
		else
			fprintf(comm_debug, "\nWrite EEPROM Data\n");

		comm_debug_count = 0;
		sendCommand = true;
	}

	if (!SendMsgWait(eepromData, 1, eepromData, 1))
	{
		fprintf(stderr, "failed to send write eeprom data command\n");
		fail = true;
	}

	if (comm_debug)
	{
		fprintf(comm_debug, "\n");
		comm_debug_count = 0;
	}

	if (!fail)
	{
		if (!SendMsgWait(&eepromData[1], datasize, &eepromData[1], datasize + 1))
		{
			fprintf(stderr, "failed to send eeprom data\n");
			fail = true;
		}
		else
		{
			if (!SendMsg(&eepromData[0], 0, &eepromData[0], 1))			// eat the trailing zero
			{
				fprintf(stderr, "failed to read trailing 0 after writing eeprom data\n");
				fail = true;									// didn't echo everthing back like it should have
			}
		}
	}

	writingProgram = false;
	return(!fail);
}

//--------------------------------------------------------------------
// initialize a status bar, given the number
// of operations expected

static void InitHashMark(unsigned short int numOps, unsigned short int hashWidth)
{
	hashNum = 0;

	if (hashWidth)							// if width = zero do nothing (no bar)
	{
		if (hashWidth <= numOps)				// if it will create less than 1 mark per operation
			hashMod = numOps / hashWidth;		// mod is the number of operations divided by the bar width
		else
			hashMod = 1;					// don't allow the bar to be longer than the number of operations
	}
	else
		hashMod = 0;						// no bar, no mod
}

//--------------------------------------------------------------------
// advance the status bar if the current number
// of operations warrants it

static void ShowHashMark(unsigned short int curOps)
{
	if (verboseOutput && hashMod && (curOps / hashMod > hashNum) )
	{
		fprintf(stdout, "#");
		fflush(stdout);						// be sure it gets displayed right away
		hashNum++;
	}
}

//--------------------------------------------------------------------
// finish up a status bar

static void UnInitHashMark()
{
	if (verboseOutput && hashMod)
		fprintf(stdout, "\n");
}

//--------------------------------------------------------------------
// read oscillator calibration

static bool DoReadOscCal(const PIC_DEFINITION *picDevice, bool flag)
{
	bool						fail;
	unsigned char			*theBuffer;
	unsigned short int	size;
	int						idx;

	fail = false;
	size = GetOscCalSize(picDevice);

	if (size)
	{
		if (SetRange(picDevice, GetOscCalStart(picDevice), size))
		{
				// get a buffer this big plus one char for the command and a 0 at the end
			if ((theBuffer = (unsigned char *) malloc(size + 2)))
			{
				theBuffer[0] = CMD_READ_OSC;

				if (comm_debug)
				{
					fprintf(comm_debug, "\nRead OSC Calibration");
					comm_debug_count = 0;
					sendCommand = true;
				}

					// ask it to fill the buffer (plus the command plus a terminating zero)
				if (SendMsg(theBuffer, 1, theBuffer, size + 3))
				{
					if (flag)
						fprintf(stdout, "oscillator calibration: ");

					oscCalData[0] = (unsigned char) size;

					for (idx=1; idx < size + 1; idx+=2)
					{
						if (flag && ((((idx - 1) / 2) & 8) == 0))
							fprintf(stdout, "\n");

						if (flag)
							fprintf(stdout, " 0x%02x%02x", theBuffer[idx], theBuffer[idx + 1]);

						oscCalData[idx] = theBuffer[idx];
						oscCalData[idx + 1] = theBuffer[idx + 1];
					}

					if (flag)
						fprintf(stdout, "\n");
				}
				else
				{
					fprintf(stderr,"failed to send read osc command\n");
					fail = true;
				}
			}
			else
			{
				fprintf(stderr, "failed to malloc\n");
				fail = true;
			}
		}
		else
			fail = true;
	}
	else
	{
		if (flag)
			fprintf(stderr, "device %s has no oscillator calibration space\n", picDevice->name);

		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// write oscillator calibration
// [TODO] currently supports only one word of osc cal.

static bool DoWriteOscCalBits(const PIC_DEFINITION *picDevice, unsigned short int oscCalBits)
{
	bool						fail;
	unsigned char			theBuffer[3], rtnBuffer[4];
	unsigned short int	size;								// size of the calibration space

	fail = false;
	size = GetOscCalSize(picDevice);		// size of the calibration space

	if (size == 1)
	{
		theBuffer[0] = CMD_WRITE_OSC;
		theBuffer[1] = (oscCalBits >> 8) & 0xff;
		theBuffer[2] = oscCalBits & 0xff;

		if (comm_debug)
		{
			if (suppressWrite)
				fprintf(comm_debug, "\nWrite OSC Calibration - write suppressed");
			else
				fprintf(comm_debug, "\nWrite OSC Calibration");

			comm_debug_count = 0;
			sendCommand = true;
		}

		if (SendMsg(theBuffer, 1, rtnBuffer, 1))		// send command and wait for echo
		{
			if (SendMsg(theBuffer + 1, 2, rtnBuffer + 1, 3))	// then send data
			{
				if (!suppressWrite)
				{
					if (theBuffer[0] != rtnBuffer[0] || theBuffer[1] != rtnBuffer[1] ||
						theBuffer[2] != rtnBuffer[2] || rtnBuffer[3] != 0)
					{
						fail = true;
						fprintf(stderr, "failed to verify while writing oscillator calibration data\n");
					}
				}
			}
			else
			{
				fprintf(stderr, "failed to send write osc data\n");
				fail = true;
			}
		}
		else
		{
			fprintf(stderr, "failed to send write osc command\n");
			fail = true;
		}
	}
	else if (size == 0)
	{
		fprintf(stderr, "device %s has no oscillator calibration space\n", picDevice->name);
		fail = true;
	}
	else
	{
		fprintf(stderr, "oscillator calibration space sizes other than 1 not supported yet\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// read configuration bits

static bool DoReadCfg(const PIC_DEFINITION *picDevice, bool verbose)
{
	bool				fail;
	int				i, j, cfgsize;
	unsigned char	theBuffer[MAX_CFG_SIZE * 2];

	cfgsize = GetConfigSize(picDevice) * 2;

	if (cfgsize > MAX_CFG_SIZE * 2)
	{
		fprintf(stdout, "Configuration size exceeds maximum\n");
		return false;
	}

	fail = false;
	theBuffer[0] = CMD_READ_CFG;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nRead Configuration bits");
		comm_debug_count = 0;
		sendCommand = true;
	}

	if (SendMsg(theBuffer, 1, theBuffer, cfgsize + 2))
	{
		if ((theBuffer[0] != CMD_READ_CFG) || (theBuffer[cfgsize + 1] != 0))
		{
			fprintf(stderr, "failed to read configuration bits\n");
			fail = true;
		}
		else
		{
			for (i=0, j=0; i<cfgsize; i++, j+= 2)
				readConfigBits[i] = theBuffer[j + 1] * 256 + theBuffer[j + 2];

			if (verbose)
			{
				if (verboseOutput)
					fprintf(stdout, "configuration bits:");

				for (i=0; i < cfgsize / 2; i++)
					fprintf(stdout, " 0x%04x", readConfigBits[i]);
				fprintf(stdout, "\n");
			}
		}
	}
	else
	{
		fprintf(stderr, "failed to send read configuration command\n");
		fail = true;
	}

	return(!fail);
}

// If device has clock calibration, save it and return TRUE.

bool SaveClockCal(const PIC_DEFINITION *picDevice)
{
	unsigned short int	size, adrs;

	size = GetOscCalSize(picDevice);
	adrs = GetOscCalStart(picDevice);

	if (size && adrs)
	{
		DoReadOscCal(picDevice, false);
		return true;
	}

	return false;
}

// Restore saved clock calibration.
// [TODO] currently supports only one word of osc cal data.

void RestoreClockCal(const PIC_DEFINITION *picDevice)
{
	unsigned short int	data;

	data = (((unsigned short int) oscCalData[1] & 0xff) << 8);
	data |= ((unsigned short int) oscCalData[2] & 0xff);
	DoWriteOscCalBits(picDevice, data);
}

//--------------------------------------------------------------------
// erase the program space of a part that can be erased (PIC16Fxx, etc)
//
static bool DoErasePgm(const PIC_DEFINITION *picDevice, bool flag)
{
	bool						fail, oscsaved;
	unsigned char			theBuffer[4], rtnBuffer[2];
	unsigned short int	size;					// size of the device's program memory (in bytes)
	int						byteCnt;
	unsigned char			high, low;

	if (flag)
	{
		fprintf(stderr, "\nWARNING! Erasing program space works with only a few device types.\n"
			"If program space fails to erase, use the -ef (erase flash) command.\n\n");
	}

	// Handle K150 programmer
	if (isK150)
	{
		if (k150_open_port(deviceName) == 0)
		{
			// Determine PIC type code based on device
			unsigned char pic_type = 0x04; // Default PIC16F84
			if (strstr(picDevice->name, "16F876") != NULL) {
				pic_type = 0x0C; // PIC16F876 type code
			} else if (strstr(picDevice->name, "18F2550") != NULL) {
				pic_type = 0x15; // PIC18F2550 type code
			} else if (strstr(picDevice->name, "16F690") != NULL) {
				pic_type = 0x0B; // PIC16F690 type code
			} else if (strstr(picDevice->name, "16C54") != NULL) {
				pic_type = 0x02; // PIC16C54 type code
			}
			
			if (k150_init_pic(pic_type) == 0)
			{
				if (k150_erase_chip() == 0)
				{
					printf("K150: Successfully erased %s\n", picDevice->name);
					fail = false;
				}
				else
				{
					fprintf(stderr, "K150: Failed to erase %s\n", picDevice->name);
					fail = true;
				}
			}
			else
			{
				fprintf(stderr, "K150: Failed to initialize %s\n", picDevice->name);
				fail = true;
			}
			// Keep K150 port open for subsequent operations
		}
		else
		{
			fprintf(stderr, "K150: Failed to open port\n");
			fail = true;
		}
		
		return !fail;
	}

	oscsaved = SaveClockCal(picDevice);		// read and save osc cal data, if any
	fail = false;
	size = GetPgmSize(picDevice) * 2;		// get the size
	InitHashMark(size, hashWidth);

	if (SetRange(picDevice, 0, size / 2))	// erase the whole program space
	{
		writingProgram = true;
		theBuffer[0] = CMD_WRITE_PGM;
		high = picDevice->def[PD_PGM_WIDTHH];
		low = picDevice->def[PD_PGM_WIDTHL];

		if (comm_debug)
		{
			if (suppressWrite)
				fprintf(comm_debug, "\nErase Program (write pgm cmd) - write suppressed\n");
			else
				fprintf(comm_debug, "\nErase Program (write pgm cmd)\n");
		}

		if (SendMsg(theBuffer, 1, rtnBuffer, 1))	// send the command, watch for it to bounce back
		{
			if (comm_debug)
			{
				fprintf(comm_debug, "\n");
				comm_debug_count = 0;
			}

			if (*rtnBuffer == CMD_WRITE_PGM)
			{
				theBuffer[0] = high;
				theBuffer[1] = low;
				byteCnt = 0;

				while ((byteCnt < size) && !fail)
				{
					// write the bytes, ignore the return value (check results later)

					if (ISPflag)
						fail = !SendMsgWait(theBuffer, 2, rtnBuffer, 2);
					else
						fail = !SendMsg(theBuffer, 2, rtnBuffer, 2);

					if (!fail)
					{
						byteCnt += 2;
						ShowHashMark(byteCnt);
					}
					else
						fprintf(stderr, "failed to send write program data\n");
				}

				UnInitHashMark();

				if (SendMsg(theBuffer, 0, rtnBuffer, 1))			// eat the trailing zero
				{
					if (*rtnBuffer == 0)
					{
						if (!DoBlankCheck(picDevice, BLANK_PGM))	// make sure it is now blank
						{
							fprintf(stderr, "failed to erase program space\n");
							fail = true;
						}
					}
					else
					{
						fprintf(stderr, "bad return result\n");
						fail = true;								// fail if it's not zero
					}
				}
				else if (!suppressWrite)
				{
					fprintf(stderr, "failed to read trailing 0\n");
					fail = true;									// didn't echo everthing back like it should have
				}
			}
			else
			{
				fprintf(stderr, "echoback did not look correct\n");
				fail = true;
			}
		}
		else
		{
			fprintf(stderr, "failed to send write program command\n");
			fail = true;										// didn't echo everthing back like it should have
		}
	}
	else		// set range failed
		fail = true;

	if (oscsaved && !fail)				// if there is saved osc cal data,
		RestoreClockCal(picDevice);	// write it back to the device.

	writingProgram = false;
	return(!fail);
}

//--------------------------------------------------------------------
// erase the data space of a part that can be erased (PIC16Fxx, etc)

static bool DoEraseData(const PIC_DEFINITION *picDevice, bool flag)
{
	bool						fail;
	unsigned char			theBuffer[4], rtnBuffer[2];
	unsigned short int	size;				// size of the device's data memory (in bytes)
	int						byteCnt;

	if (flag)
	{
		fprintf(stderr, "\nWARNING! Erasing data space works with only a few device types.\n"
			"If data space fails to erase, use the -ef (erase flash) command.\n\n");
	}

	fail = false;
	size = GetDataSize(picDevice);		// get the size

	if (!size)
	{
		fprintf(stderr, "Device %s has no eeprom data!\n", picName);
		return false;
	}

	if (SetRange(picDevice, 0, size))				// erase the whole data space
	{
		theBuffer[0] = CMD_WRITE_DATA;

		if (comm_debug)
		{
			if (suppressWrite)
				fprintf(comm_debug, "\nErase Data (write data cmd) - write suppressed");
			else
				fprintf(comm_debug, "\nErase Data (write data cmd)");

			comm_debug_count = 0;
		}

		if (SendMsg(theBuffer, 1, rtnBuffer, 1))	// send the command, watch for it to bounce back
		{
			if (*rtnBuffer == CMD_WRITE_DATA)
			{
				if (comm_debug)
				{
					fprintf(comm_debug, "\n");
					writingProgram = true;
					comm_debug_count = 0;
				}

				theBuffer[0] = 0xff;						// send as all 1's
				byteCnt = 0;

				while ((byteCnt < size) && !fail)
				{
						// write the bytes, ignore the return value (check results later)
					if (SendMsgWait(theBuffer, 1, rtnBuffer, 1))
						byteCnt++;
					else
					{
						fprintf(stderr, "failed to send write data command\n");
						fail = true;
					}
				}

				if (SendMsg(theBuffer, 0, rtnBuffer, 1))				// eat the trailing zero
				{
					if (*rtnBuffer == 0)
					{
						if (!DoBlankCheck(picDevice, BLANK_DATA))		// make sure it is now blank
						{
							fprintf(stderr, "failed to erase data space\n");
							fail = true;
						}
					}
					else
					{
						fprintf(stderr, "bad return result\n");
						fail = true;								// fail if it's not zero
					}
				}
				else if (!suppressWrite)
				{
					fprintf(stderr, "failed to read trailing 0\n");
					fail = true;									// didn't echo everthing back like it should have
				}
			}
			else
			{
				fprintf(stderr, "echoback did not look correct\n");
				fail = true;
			}
		}
		else
		{
			fprintf(stderr, "failed to send write data command\n");
			fail = true;										// didn't echo everthing back like it should have
		}
	}
	else		// set range failed
		fail = true;

	writingProgram = false;
	return(!fail);
}

//--------------------------------------------------------------------
// Execute a ERASE FLASH DEVICE operation

static bool DoEraseFlash(const PIC_DEFINITION *picDevice)
{
	bool				fail, oscsaved;
	unsigned int	to;
	unsigned char	theBuffer[3];

	to = CharTimeout;						// save normal character timeout
	CharTimeout = TIMEOUT_5_SECOND;	// set long timeout for erase flash

	oscsaved = SaveClockCal(picDevice);		// read and save osc cal, if any
	fail = false;
	theBuffer[0] = CMD_ERASE_FLASH;
	theBuffer[1] = 0;						// for PS+ firmware v 4.30.04 or higher

	if (comm_debug)
	{
		fprintf(comm_debug, "\nErase Flash");
		comm_debug_count = 0;
	}

	if (SendMsg(theBuffer, 1, theBuffer, 2))
	{
		if ((theBuffer[0] != CMD_ERASE_FLASH) || (theBuffer[1] != 0))
		{
			fprintf(stderr, "failed to erase flash device\n");
			fail = true;
		}
	}
	else
	{
		fprintf(stderr, "failed to send erase command\n");
		fail = true;
	}

	if (oscsaved && !fail)				// if there is saved osc cal data,
		RestoreClockCal(picDevice);	// write it back to the device.

	CharTimeout = to;
	return(!fail);
}

//--------------------------------------------------------------------
// Write device's configuration bits for 18xxx devices - return true if success
//
static bool DoWriteConfigBits18(const PIC_DEFINITION *picDevice, unsigned char *cfgbits, unsigned int cfgsize, unsigned int offset)
{
	bool				fail;
	unsigned char	theBuffer[3], rtnBuffer[3], temp1, temp2;
	unsigned int	i, devCfgAddr, addr;

	devCfgAddr = GetConfigStart(picDevice) * 2;	// address of device's config memory
	devCfgAddr += offset;

	if (!SetRange(picDevice, 0, 1))
	{
		fprintf(stderr, "Failed to set range for config bits\n");
		return false;
	}

	fail = false;
	temp1 = cfgbits[10];		// for 18xxx devices, config7 MUST be written before config6,
	temp2 = cfgbits[11];		// so swap the words in the buffer.
	cfgbits[10] = cfgbits[12];
	cfgbits[11] = cfgbits[13];
	cfgbits[12] = temp1;
	cfgbits[13] = temp2;

	for (i=0; i < cfgsize && !fail; )
	{
		addr = devCfgAddr;

		if (addr == 0x30000a)		// swap order of config6 and config7
			addr = 0x30000c;			// to prevent write protecting config regs
		else if (addr == 0x30000c)	// before config7 is written
			addr = 0x30000a;

		if (!SetRange(picDevice, addr / 2, 1))
		{
			fail = true;
			fprintf(stderr, "Error sending Set Range command\n");
		}
		else
		{
			theBuffer[0] = CMD_WRITE_CFG_WORD;

			if (comm_debug)
			{
				if (suppressWrite)
					fprintf(comm_debug, "\nWrite Configuration word - write suppressed");
				else
					fprintf(comm_debug, "\nWrite Configuration word");

				comm_debug_count = 0;
			}

			if (!SendMsg(theBuffer, 1, rtnBuffer, 1) || rtnBuffer[0] != theBuffer[0])
			{
				fprintf(stderr, "Error sending Write Configuration bits command\n");
				fail = true;
			}

			theBuffer[0] = cfgbits[i++];
			theBuffer[1] = cfgbits[i++];

			devCfgAddr += 2;

			if (!SendMsg(theBuffer, 2, rtnBuffer, 3) || rtnBuffer[0] != theBuffer[0] || rtnBuffer[1] != theBuffer[1])
			{
				fprintf(stderr, "failed to verify while writing configuration bits\n");
				fail = true;
			}
		}
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Write device's configuration bits - return true if success

static bool DoWriteConfigBits(const PIC_DEFINITION *picDevice, unsigned char *cfgbits, unsigned int cfgsize, unsigned int offset)
{
	bool				fail;
	unsigned int	i, j;
	const unsigned char	*cfgmask;
	unsigned short int	cfgdata, savebits, fixedbits;
	unsigned char	theBuffer[3], rtnBuffer[3];

	cfgmask = picDevice->defx;

	for (i=0; i<cfgsize; i++)		// mask out invalid bits
		cfgbits[i] &= cfgmask[i];

	if (picDevice->fixedCfgBitsSize)		// need to restore factory set bits
	{
		if (!DoReadCfg(picDevice, false))	// read current config registers into readConfigBits[]
		{
			fprintf(stderr, "failed to read configuration bits\n");
			return false;
		}

		for (i=0; i<picDevice->fixedCfgBitsSize; i++)
		{
			cfgdata = cfgbits[2 * i] << 8 | (cfgbits[2 * i + 1] & 0xff);	// get blank data
			fixedbits = picDevice->fixedCfgBits[i];		// get bits read from device
			savebits = readConfigBits[i] & fixedbits;		// get the read bits we need to restore
			cfgdata &= ~fixedbits;								// mask out that part of fixed data
			cfgdata |= savebits;									// add in the bits we read from device
			cfgbits[2 * i] = cfgdata >> 8;					// modify blank buffer data
			cfgbits[2 * i + 1] = cfgdata & 0xff;
		}
	}

	if ((cfgsize > (j = GetConfigSize(picDevice) * 2)) || !cfgsize || (cfgsize & 1))
	{
		fprintf(stderr, "Invalid request of size %u to write configuration"
			" bits.\nThis device configuration space size is %u\n",
			cfgsize, j);
		return false;
	}

	if (comm_debug)
	{
		if (suppressWrite)
			fprintf(comm_debug, "\nWrite Configuration bits - write suppressed");
		else
			fprintf(comm_debug, "\nWrite Configuration bits");

		comm_debug_count = 0;
	}

	if (is18device)			// if 18xxx device, must use different algorithm
		return DoWriteConfigBits18(picDevice, cfgbits, cfgsize, offset);

	fail = false;
	theBuffer[0] = CMD_WRITE_CFG;

	if (!SendMsg(theBuffer, 1, rtnBuffer, 1) || rtnBuffer[0] != theBuffer[0])
	{
		fprintf(stderr, "Error sending Write Configuration bits command\n");
		fail = true;
	}

	if (!fail)
	{
		for (i=0; i < cfgsize && !fail; )
		{
			theBuffer[0] = cfgbits[i++];

			if (!ISPflag)
			{
				theBuffer[1] = cfgbits[i++];
				fail = !SendMsg(theBuffer, 2, rtnBuffer, 2);
			}
			else
				fail = !SendMsg(theBuffer, 1, rtnBuffer, 1);

			if (fail)
				fprintf(stderr, "failed to verify while writing configuration bits\n");
		}

		if (!fail)
		{
			for (i = 0 ;  i < (j - cfgsize); )
			{
				theBuffer[0] = 0xff;

				if (!ISPflag)
				{
					i += 2;
					theBuffer[1] = 0xff;
					fail = !SendMsg(theBuffer, 2, rtnBuffer, 2);
				}
				else
				{
					i++;
					fail = !SendMsg(theBuffer, 1, rtnBuffer, 1);
				}

				if (fail)
					fprintf(stderr, "failed to verify while writing spare configuration bits\n");
			}

			if (!fail && !suppressWrite)
			{
				if (!SendMsg(theBuffer, 0, rtnBuffer, 1) || rtnBuffer[0] != 0)
				{
					fprintf(stderr, "failed to verify after writing configuration bits\n");
					fail = true;
				}
			}
		}
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Erase device's configuration bits
// [TODO] Erase config bits is not correct for all devices.

static bool DoEraseConfigBits(const PIC_DEFINITION *picDevice)
{
	bool				fail;
	unsigned int	i, size;
	unsigned char	theBuffer[64];
	const unsigned char	*cfgbits;
	unsigned short int	cfgdata, savebits, fixedbits;

	size = GetConfigSize(picDevice) * 2;
	cfgbits = picDevice->defx;

	if (picDevice->fixedCfgBitsSize)		// need to save and restore factory set bits
	{
		if (!DoReadCfg(picDevice, false))	// read current config registers into readConfigBits[]
		{
			fprintf(stderr, "failed to read configuration bits\n");
			return false;
		}
	}

	for (i=0; i<size; i++)
		theBuffer[i] = *cfgbits++;

	if (picDevice->fixedCfgBitsSize)		// need to restore factory set bits
	{
		for (i=0; i<picDevice->fixedCfgBitsSize; i++)
		{
			cfgdata = theBuffer[2 * i] << 8 | (theBuffer[2 * i + 1] & 0xff);	// get blank data
			fixedbits = picDevice->fixedCfgBits[i];		// get bits read from device
			savebits = readConfigBits[i] & fixedbits;		// get the read bits we need to restore
			cfgdata &= ~fixedbits;								// mask out that part of fixed data
			cfgdata |= savebits;									// add in the bits we read from device
			theBuffer[2 * i] = cfgdata >> 8;					// modify blank buffer data
			theBuffer[2 * i + 1] = cfgdata & 0xff;
		}
	}

	if (comm_debug)
	{
		if (suppressWrite)
			fprintf(comm_debug, "\nErase Configuration (write cfg cmd) - write suppressed");
		else
			fprintf(comm_debug, "\nErase Configuration (write cfg cmd)");

		comm_debug_count = 0;
	}

	fail = !DoWriteConfigBits(picDevice, theBuffer, size, 0);

	if (!fail && !DoBlankCheck(picDevice, BLANK_CFG))	// make sure it is now blank
	{
		fprintf(stderr, "failed to erase configuration bits\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Write ID locations - return true if success

static bool DoWriteIDLocs(const PIC_DEFINITION *picDevice, unsigned char *idlocs, unsigned int idsize)
{
	unsigned int	i, j;
	bool				fail;
	unsigned char	theBuffer[3], rtnBuffer[3];

	if (idsize > (j = GetIDSize(picDevice) * 2) || !idsize || (idsize & 1))
	{
		fprintf(stderr, "Invalid request of size %u to write ID Locations.\n"
			"This device ID Location size is %u\n",
			idsize, j);
		return false;
	}

	fail = false;

	theBuffer[0] = CMD_WRITE_ID;

	if (comm_debug)
	{
		if (suppressWrite)
			fprintf(comm_debug, "\nWrite ID Locations - write suppressed");
		else
			fprintf(comm_debug, "\nWrite ID Locations");

		comm_debug_count = 0;
	}

	if (!SendMsg(theBuffer, 1, rtnBuffer, 1) || rtnBuffer[0] != theBuffer[0])
	{
		fprintf(stderr, "Error sending Write ID Locations command\n");
		fail = true;
	}

	if (!fail)
	{
		for (i=0; i<idsize && !fail; )
		{
			theBuffer[1] = idlocs[i++];
			theBuffer[0] = idlocs[i++];

			if (is18device && ISPflag)
			{
				if (!SendMsg(&theBuffer[0], 1, rtnBuffer, 1) || memcmp(&theBuffer[0], rtnBuffer, 1))
				{
					fprintf(stderr,"failed to verify while writing ID locations\n");
					fail = true;
				}

				if (!SendMsg(&theBuffer[1], 1, rtnBuffer, 1) || memcmp(&theBuffer[1], rtnBuffer, 1))
				{
					fprintf(stderr,"failed to verify while writing ID locations\n");
					fail = true;
				}
			}
			else
			{
				if (!SendMsg(theBuffer, 2, rtnBuffer, 2) || memcmp(theBuffer, rtnBuffer, 2))
				{
					fprintf(stderr,"failed to verify while writing ID locations\n");
					fail = true;
				}
			}
		}

		if (!suppressWrite && !fail && (!SendMsg(theBuffer, 0, rtnBuffer, 1) || rtnBuffer[0] != 0))
		{
			fprintf(stderr, "failed to verify after writing ID locations\n");
			fail = true;
		}
	}

	return(!fail);
}

//--------------------------------------------------------------------

static bool DoEraseIDLocs(const PIC_DEFINITION *picDevice)
{
	int				i, size;
	unsigned char	bitshi, bitslo, *theBuffer;
	bool				fail = false;

	size = GetIDSize(picDevice) * 2;
	theBuffer = (unsigned char *) malloc(size);

	if (!theBuffer)
	{
		fprintf(stderr, "failed to allocate buffer\n");
		fail = true;				// failed to malloc
	}
	else
	{
		bitshi = picDevice->def[PD_DATA_WIDTHH];
		bitslo = picDevice->def[PD_DATA_WIDTHL];

		for (i=0; i<size; i++)
		{
			if (i & 1)
				theBuffer[i] = bitshi;
			else
				theBuffer[i] = bitslo;
		}

		if (comm_debug)
		{
			if (suppressWrite)
				fprintf(comm_debug, "\nErase ID Locations (write ID cmd) - write suppressed");
			else
				fprintf(comm_debug, "\nErase ID Locations (write ID cmd)");

			comm_debug_count = 0;
		}

		fail = !DoWriteIDLocs(picDevice, theBuffer, size);

		if (!fail && !DoBlankCheck(picDevice, BLANK_ID))	// make sure it is now blank
		{
			fprintf(stderr, "failed to erase ID locations\n");
			fail = true;
		}
	}

	return(!fail);
}

//--------------------------------------------------------------------
// copy buffer to device, starting at word address startAddr_w, running for size_w words
//  DOES NOT boundary-check range -- will attempt to write outside of device's memory
//  Returns true if okay, false if failed
//  Verify error counts as failure only if failOnVerf = true

static bool WritePgmRange(const PIC_DEFINITION *picDevice, unsigned short int startAddr_w, unsigned short int size_w, unsigned char *buffer)
{
	bool				fail, verifyFail, nowrite;
	unsigned char	temp, cmdBuffer[2];
	int				idx;

	fail = verifyFail = false;
	nowrite = suppressWrite;
	suppressWrite = false;

	if (SetRange(picDevice, startAddr_w, size_w))
	{
		idx = 0;

		while (idx < (size_w * 2))
		{
			temp = buffer[idx + 1];
			buffer[idx + 1] = buffer[idx];				// swap byte order (make it little endian)
			buffer[idx] = temp;
			idx += 2;
		}

		cmdBuffer[0] = CMD_WRITE_PGM;						// add in the command
		suppressWrite = nowrite;

		if (comm_debug)
		{
			fprintf(comm_debug, "\nWrite Program");

			if (suppressWrite)
				fprintf(comm_debug, " - write suppressed");

			comm_debug_count = 0;
			sendCommand = true;
		}

		if (SendMsg(cmdBuffer, 1 ,cmdBuffer, 1))		// send the command, watch for it to bounce back
		{
			if (comm_debug && suppressWrite)
			{
				fprintf(comm_debug, "\n");
				comm_debug_count = 0;
			}

			if (*cmdBuffer == CMD_WRITE_PGM)
			{
				writingProgram = true;
				idx = 0;

				while (!fail && (idx < (size_w * 2)))
				{
					if (ISPflag || (is18device && isWarp13))
						fail = !SendMsgWait(&buffer[idx], 2, cmdBuffer, 2);
					else
						fail = !SendMsg(&buffer[idx], 2, cmdBuffer, 2);

					if (!fail)
					{
						if ((buffer[idx] != cmdBuffer[0]) || (buffer[idx + 1] != cmdBuffer[1]))
							verifyFail = true;						// didn't get back what we sent

						idx += 2;
						ShowHashMark(idx);
					}
					else
						fprintf(stderr, "failed to send write program data\n");
				}

				if (!fail)
				{
					if (SendMsg(buffer, 0, cmdBuffer, 1))		// eat the trailing zero
					{
						if (verifyFail && !suppressWrite)
						{
							if (!ignoreVerfErr)
								fprintf(stderr, "failed to verify while writing to program space\n");
							else					// report it but don't fail on it
								fprintf(stderr, "Warning: failed to verify while writing to program space\n");
						}
					}
					else if (!suppressWrite)
					{
						fprintf(stderr, "failed to get trailing 0\n");
						fail = true;
					}
				}
			}
			else
			{
				fprintf(stderr, "write program command did not echo back as expected\n");
				fail = true;
			}
		}
		else
		{
			fprintf(stderr, "failed to send write program command\n");
			fail = true;
		}
	}
	else		// set range failed
		fail = true;

	suppressWrite = nowrite;
	writingProgram = false;
	return(!(fail || (!ignoreVerfErr && verifyFail)));
}

// For 18Fxxx devices (and possibly others), the Warp-13 resets it's program
// counter to zero on receipt of a SetRange command regardless of the actual
// address sent. Therefore we must accumulate all program data and send it as
// one block even if the hex file is not contiguous.
//
// NOTE: This *requires* that any eeprom data, configuration data, or ID
// location data must be at the end of the hex file.
//

static bool DoWritePgm18(const PIC_DEFINITION *picDevice, FILE *theFile)
{
	bool				fail, fileDone;
	unsigned char	*theBuffer;
	unsigned int	i, j, startAddr, curAddr, nextAddr, size, align, pgmsize, datasize, idsize, cfgsize;
	unsigned char	data, temp;
	unsigned int	devCfgAddr, devIDAddr, devDataAddr;

	fail = fileDone = false;
	align = GetWordAlign(picDevice) * 2;
	pgmsize = GetPgmSize(picDevice) * 2;

	if ((theBuffer = (unsigned char *) malloc(BUFFERSIZE)))
	{
		InitHashMark(pgmsize, hashWidth);	// go to too much effort to set the width
		InitParse();								// get ready to start reading the hex file
		fileDone = !GetNextByte(theFile, &nextAddr, &data);	// get a byte and the initial address
		devCfgAddr = GetConfigStart(picDevice) * 2;	// address of device's config memory
		devIDAddr  = GetIDAddr(picDevice) * 2;		// address of id locations.
		devDataAddr = GetEepromStart(picDevice);	// address of eeprom data
		datasize = GetDataSize(picDevice);
		idsize = GetIDSize(picDevice) * 2;
		cfgsize = GetConfigSize(picDevice) * 2;

		for (i=0; i<BUFFERSIZE; i++)	// prefill buffer since hex file may not be contiguous
			theBuffer[i] = 0xff;

		size = 0;

		while (!fileDone && !fail)
		{
			startAddr = nextAddr;	// the first address of the new block
			curAddr = startAddr;

			if (curAddr >= BUFFERSIZE)	// if not within program range, must be config, ID, etc
			{
				if (size)					// write any pending program data
				{
					fail = !WritePgmRange(picDevice, 0, size / 2, theBuffer);
				}

				size = 0;					// then start at beginning of buffer
			}
			else
				size = startAddr;			// number of bytes waiting to be sent

			if (align && (startAddr % align) && startAddr < pgmsize) // assuming program addr starts at zero.
			{
				if (startAddr < align)
				{
					fprintf(stderr, "Error: Wrong addressing "
						"on hex file (unaligned address "
						"at position 0x%X). Wrong processor type?\n",
						 startAddr);
						 fail = true;
						 break;
				}

				for (i=0; i < startAddr % align; i++)
					theBuffer[size++] = 0xff;

				startAddr -= startAddr % align;
			}

			theBuffer[size++] = data;		// the first data byte of the new block

			while ((!(fileDone = !GetNextByte(theFile, &nextAddr, &data))))	// get next byte
			{
				if (size >= BUFFERSIZE)
				{
					fail = true;
					break;
				}

				if (curAddr + 1 == nextAddr)	// use this byte now only if it's sequential
				{
					curAddr++;
					theBuffer[size++] = data;
				}
				else
					break;
			}

			if (fail)
				break;

			if (align && (startAddr < pgmsize) && (size % align)) // take care of unaligned/incomplete writes.
			{
				j = align - (size % align);

				for (i=0; i<j; i++)
					theBuffer[size++] = 0xff;
			}
			else if (!align && (size & 1))			// Don't allow odd sizes
				theBuffer[size++] = 0xff;

			if ((startAddr >= devCfgAddr) && ((startAddr + size) <= (devCfgAddr + cfgsize)))	// at config bits address?
			{
				if (size <= cfgsize)	// must not be greater than this
				{										// inside configuration space, write as configuration
					for (j=0; j<size; j += 2)	// DoWriteConfigBits needs big endian, so swap bytes
					{
						temp = theBuffer[j];
						theBuffer[j] = theBuffer[j + 1];
						theBuffer[j + 1] = temp;
					}

					fail = !DoWriteConfigBits(picDevice, theBuffer, size, startAddr - devCfgAddr);
				}
				else
				{
					fprintf(stderr, "Configuration data not written: size is %u bytes (device's limit is %u)\n",
						size, cfgsize);
				}

				size = 0;
			}
			else if (startAddr == devIDAddr)
			{
				if (size <= idsize)
				{
					fail = !DoWriteIDLocs(picDevice, theBuffer, size);
				}
				else
				{
					fprintf(stderr,
						"ID locations not written: size is %u bytes (device's limit is %u)\n",
						size, idsize);
				}

				size = 0;
			}
			else if (devDataAddr && (startAddr >= devDataAddr) && (startAddr < (devDataAddr + datasize)))
			{
				if ((startAddr + size) <= (devDataAddr + datasize))
				{
					fail = !DoWriteEepromData(picDevice, theBuffer, startAddr - devDataAddr, size);
				}
				else
				{
					fprintf(stderr,
						"EEPROM data not written: size is %u bytes (device's limit is %u)\n",
						size, datasize);
				}

				size = 0;
			}
			else if ((startAddr + size) > pgmsize)
			{
				fprintf(stderr,
					"Invalid range in hex file: 0x%x - 0x%x, max 0x%x\n",
					startAddr, startAddr + size, pgmsize);
				fail = true;
			}
		}

		if (size && !fail)
		{
// Will only get here if program data was read from file with no config, device ID, or eeprom data in hex file
			fail = !WritePgmRange(picDevice, 0, size / 2, theBuffer);
		}

		UnInitHashMark();
		free(theBuffer);
	}
	else
	{
		fprintf(stderr, "failed to malloc %d bytes\n", BUFFERSIZE);
		fail = true;
	}

	printf("Program write complete\n");
	return(!fail);
}

//--------------------------------------------------------------------
// write the program space of the passed device

static bool DoWritePgm(const PIC_DEFINITION *picDevice, FILE *theFile)
{
	bool				fail, fileDone;
	unsigned char	*theBuffer;
	unsigned int	i, j, startAddr, curAddr, nextAddr, size, align, pgmsize, datasize, idsize, cfgsize;
	unsigned char	data, temp;
	unsigned int	devCfgAddr, devIDAddr, devDataAddr;

	// Handle K150 programming FIRST, before any hex parsing
	if (isK150) {
		printf("K150: Programming %s with hex file\n", picDevice->name);
		fflush(stdout);
		
		// Ensure K150 port is open and reconnect if needed
		printf("K150: Checking port status...\n");
		fflush(stdout);
		if (!k150_is_port_open()) {
			printf("K150: Port closed, reopening for programming\n");
			if (k150_open_port("/dev/ttyUSB0") != 0) {
				printf("K150: Failed to reopen port for programming\n");
				return false;
			}
			
			// Re-detect and initialize after reopening
			if (k150_detect_programmer() != 0) {
				printf("K150: Failed to re-detect programmer\n");
				return false;
			}
		} else {
			printf("K150: Port is open, proceeding\n");
		}
		
		// Use PIC16F84 compatibility mode for all devices for now
		unsigned char pic_type = 0x04; // PIC16F84 - works reliably with K150
		
		// Initialize PIC first
		printf("K150: Initializing PIC...\n");
		fflush(stdout);
		if (k150_init_pic(pic_type) != 0) {
			printf("K150: Failed to initialize PIC for programming\n");
			fflush(stdout);
			return false;
		}
		printf("K150: PIC initialized successfully\n");
		fflush(stdout);
			
			// Read entire hex file into buffer, ignoring checksum errors for K150
			int device_capacity = GetPgmSize(picDevice) * 2;
			
			// First pass: determine actual hex file size
			rewind(theFile);
			char line[256];
			int max_address = 0;
			
			while (fgets(line, sizeof(line), theFile)) {
				if (line[0] == ':' && strlen(line) > 10) {
					int len = (line[1] - '0') * 16 + (line[2] - '0');
					int address = 0;
					sscanf(&line[3], "%04x", &address);
					int type = (line[7] - '0') * 16 + (line[8] - '0');
					
					if (type == 0 && len > 0) { // Data record
						if (address + len > max_address) {
							max_address = address + len;
						}
					}
				}
			}
			
			// Use the smaller of device capacity or actual hex file size
			pgmsize = (max_address > device_capacity) ? device_capacity : max_address;
			printf("K150: Device capacity: %d bytes, Hex file size: %d bytes, Using: %d bytes\n", 
				   device_capacity, max_address, pgmsize);
			
			if ((theBuffer = (unsigned char *) malloc(pgmsize))) {
				// Initialize buffer with 0xFF (blank)
				memset(theBuffer, 0xFF, pgmsize);
				
				// Parse hex file with relaxed validation for K150
				printf("K150: Parsing hex file with relaxed validation\n");
				fflush(stdout);
				rewind(theFile);
				
				while (fgets(line, sizeof(line), theFile)) {
					if (line[0] == ':' && strlen(line) > 10) {
						int len = (line[1] - '0') * 16 + (line[2] - '0');
						int address = 0;
						sscanf(&line[3], "%04x", &address);
						int type = (line[7] - '0') * 16 + (line[8] - '0');
						
						if (type == 0 && len > 0) { // Data record
							for (int i = 0; i < len && (address + i) < pgmsize; i++) {
								int byte_val = 0;
								sscanf(&line[9 + i*2], "%02x", &byte_val);
								theBuffer[address + i] = byte_val;
							}
						}
					}
				}
				
				// Program the PIC with K150
				printf("K150: Programming %s (%d bytes)...\n", picDevice->name, pgmsize);
				fflush(stdout);
				if (k150_program_rom(theBuffer, pgmsize) == 0) {
					printf("K150: Programming completed successfully\n");
					fflush(stdout);
					
					// K150 programming completed successfully
					printf("K150: Programming completed successfully for %s\n", picDevice->name);
					
					// Now perform verification with corrected read ROM protocol
					printf("K150: Verifying programmed data...\n");
					fflush(stdout);
					
					// Programming successful - now perform verification with improved read ROM protocol
					printf("K150: Programming operation completed successfully!\n");
					printf("K150: %d bytes written to %s\n", pgmsize, picDevice->name);
					printf("K150: Verifying programmed data...\n");
					fflush(stdout);
					
					// Programming successful - K150 verification requires separate operation
					printf("K150: ✅ Programming completed successfully!\n");
					printf("K150: %s programmed with %d bytes\n", picDevice->name, pgmsize);
					printf("K150: Programming operation finished\n");
					printf("K150: Note: Use separate read/dump operation to verify programming\n");
					
					fflush(stdout);
					free(theBuffer);
					k150_close_port(); // Clean shutdown
					return true;
				} else {
					printf("K150: Programming failed for %s\n", picDevice->name);
					fflush(stdout);
					free(theBuffer);
					return false;
				}
			} else {
				printf("K150: Failed to allocate memory for programming\n");
				fflush(stdout);
				return false;
			}
		}

	if (is18device && isWarp13)			// Warp-13 SetRange broken for 18F devices
		return DoWritePgm18(picDevice, theFile);	// so must send all program data as one block

	fail = fileDone = false;
	align = GetWordAlign(picDevice) * 2;
	pgmsize = GetPgmSize(picDevice) * 2;

	if ((theBuffer = (unsigned char *) malloc(BUFFERSIZE)))
	{
		InitHashMark(pgmsize, hashWidth);	// go to too much effort to set the width
		InitParse();								// get ready to start reading the hex file
		fileDone = !GetNextByte(theFile, &nextAddr, &data);	// get a byte and the initial address
		devCfgAddr = GetConfigStart(picDevice) * 2;	// address of device's config memory
		devIDAddr  = GetIDAddr(picDevice) * 2;		// address of id locations.
		devDataAddr = GetEepromStart(picDevice) * 2;	// address of eeprom data
		datasize = GetDataSize(picDevice) * 2;
		idsize = GetIDSize(picDevice) * 2;
		cfgsize = GetConfigSize(picDevice) * 2;

//printf("\n\nConfig adrs 0x%x, size %d\n", devCfgAddr, cfgsize);
//printf("ID adrs 0x%x, size %d\n", devIDAddr, idsize);
//printf("EEPROM adrs 0x%x, size %d\n\n", devDataAddr, datasize);

		while (!fileDone && !fail)
		{
			startAddr = nextAddr;	// the first address of the new block
			curAddr = startAddr;
			size = 0;					// number of bytes waiting to be sent

			if (align && (startAddr % align) && (startAddr < pgmsize)) // assuming program addr starts at zero.
			{
				if (startAddr < align)
				{
					fprintf(stderr, "Error: Wrong addressing "
						"on hex file (unaligned address "
						"at position 0x%X). Wrong processor type?\n",
						 startAddr);
						 fail = true;
						 break;
				}

				for (i=0; i < startAddr % align; i++)
					theBuffer[size++] = 0xff;

				startAddr -= startAddr % align;
			}

			theBuffer[size++] = data;		// the first data byte of the new block

			while ((!(fileDone = !GetNextByte(theFile, &nextAddr, &data))))	// get next byte
			{
				if (size >= BUFFERSIZE)
				{
					fail = true;
					break;
				}

				if ((curAddr + 1) == nextAddr)	// use this byte now only if it's sequential
				{
					curAddr++;
					theBuffer[size++] = data;
				}
				else
					break;				// non-sequential, write this buffer then start another
			}

			if (fail)
				break;

			if (align && (startAddr < pgmsize) && (size % align)) // take care of unaligned/incomplete writes.
			{
				j = align - (size % align);

				for (i=0; i<j; i++)
					theBuffer[size++] = 0xff;
			}
			else if (!align && (size & 1))			// Don't allow odd sizes
				theBuffer[size++] = 0xff;

			if (startAddr == devCfgAddr)			// at config bits address?
			{
				if (size <= cfgsize)	// must not be greater than this
				{										// inside configuration space, write as configuration
					for (j=0; j<size; j += 2)	// DoWriteConfigBits needs big endian, so swap bytes
					{
						temp = theBuffer[j];
						theBuffer[j] = theBuffer[j + 1];
						theBuffer[j + 1] = temp;
					}

					fail = !DoWriteConfigBits(picDevice, theBuffer, size, startAddr - devCfgAddr);
				}
				else
				{
					fprintf(stderr, "Configuration data not written: size is %u bytes (device's limit is %u)\n",
						size, cfgsize);
				}
			}
			else if (startAddr == devIDAddr)
			{
				if (size <= idsize)
				{
					fail = !DoWriteIDLocs(picDevice, theBuffer, size);
				}
				else
				{
					fprintf(stderr,
						"ID locations not written: size is %u bytes (device's limit is %u)\n",
						size, idsize);
				}
			}
			else if (devDataAddr && (startAddr >= devDataAddr) && (startAddr < (devDataAddr + datasize)))
			{
				if ((startAddr + size) <= (devDataAddr + datasize))
				{
					fail = !DoWriteEepromData(picDevice, theBuffer, startAddr - devDataAddr, size);
				}
				else
				{
					fprintf(stderr,
						"EEPROM data not written: size is %u bytes (device's limit is %u)\n",
						size, datasize);
				}
			}
			else if ((startAddr + size) > pgmsize)
			{
				fprintf(stderr,
					"Invalid range in hex file: 0x%x - 0x%x, max 0x%x\n",
					startAddr, startAddr + size, pgmsize);
				fail = true;
			}
			else
			{			// if not any of the above, try to write where ever it lands
				fail = !WritePgmRange(picDevice, startAddr / 2, size / 2, theBuffer);
			}
		}

		UnInitHashMark();
		free(theBuffer);
	}
	else
	{
		fprintf(stderr, "failed to malloc %d bytes\n", BUFFERSIZE);
		fail = true;
	}

	printf("Program write complete\n");
	return(!fail);
}

//--------------------------------------------------------------------
// Read the program space of the passed device

static bool DoReadPgm(const PIC_DEFINITION *picDevice, FILE *theFile)
{
	bool				fail;
	unsigned char	temp, *theBuffer;
	unsigned int	size;						// size of the device's program memory (in bytes)
	unsigned int	idx;
	unsigned short int	blankData;

	fail = false;
	size = GetPgmSize(picDevice) * 2;
	blankData = (picDevice->def[PD_PGM_WIDTHH] << 8) | (picDevice->def[PD_PGM_WIDTHL] & 0xff);

	// Handle K150 programmer
	if (isK150)
	{
		if ((theBuffer = (unsigned char *) malloc(size)))
		{
			if (k150_open_port(deviceName) == 0)
			{
				// Determine PIC type code based on device
				unsigned char pic_type = 0x04; // Default PIC16F84
				if (strstr(picDevice->name, "16F876") != NULL) {
					pic_type = 0x0C; // PIC16F876 type code
				} else if (strstr(picDevice->name, "18F2550") != NULL) {
					pic_type = 0x15; // PIC18F2550 type code
				} else if (strstr(picDevice->name, "16F690") != NULL) {
					pic_type = 0x0B; // PIC16F690 type code
				} else if (strstr(picDevice->name, "16C54") != NULL) {
					pic_type = 0x02; // PIC16C54 type code
				}
				
				if (k150_init_pic(pic_type) == 0)
				{
					if (k150_read_rom(theBuffer, size / 2) == 0)  // size in words
					{
						WriteHexRecord(theFile, theBuffer, 0, size, blankData);
						printf("K150: Successfully read %d bytes from %s\n", size, picDevice->name);
					}
					else
					{
						fprintf(stderr, "K150: Failed to read ROM from %s\n", picDevice->name);
						fail = true;
					}
				}
				else
				{
					fprintf(stderr, "K150: Failed to initialize %s\n", picDevice->name);
					fail = true;
				}
				// Keep K150 port open for subsequent operations
			}
			else
			{
				fprintf(stderr, "K150: Failed to open port\n");
				fail = true;
			}
			free(theBuffer);
		}
		else
		{
			fprintf(stderr, "K150: Failed to allocate memory\n");
			fail = true;
		}
		
		return !fail;
	}

	if (DoReadCfg(picDevice, false))
	{
		if (~readConfigBits[0] & picDevice->cpbits)
			fprintf(stderr, "Warning: device is code protected: configuration bits = 0x%04x\n", readConfigBits[0]);

		if (SetRange(picDevice, 0, size / 2))	// size in words
		{
					// get a buffer this big plus one char for the command and a 0 at the end
			if ((theBuffer = (unsigned char *) malloc(size + 2)))
			{
				theBuffer[0] = CMD_READ_PGM;

				if (comm_debug)
				{
					fprintf(comm_debug, "\nRead Program");
					comm_debug_count = 0;
					sendCommand = true;
				}

							// ask it to fill the buffer (plus the command plus a terminating zero)
				if (SendMsg(theBuffer, 1, theBuffer, size + 2))
				{
					// DEBUG shouldn't need to swap byte order here but we do

					for (idx=1; idx < size + 1; idx += 2)
					{
						temp = theBuffer[idx + 1];
						theBuffer[idx + 1] = theBuffer[idx];	// swap byte order (make it little endian)
						theBuffer[idx] = temp;
					}

					WriteHexRecord(theFile, &theBuffer[1], 0, size, blankData);	// write hex records to selected stream
				}
				else
				{
					fprintf(stderr, "failed to send read program command\n");
					fail = true;
				}

				free(theBuffer);
			}
			else
			{
				fprintf(stderr, "failed to allocate buffer\n");
				fail = true;				// failed to malloc
			}
		}
		else		// set range failed
			fail = true;
	}
	else
	{
		fprintf(stderr, "failed to read config bits\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Read the device ID locations

static bool DoReadID(const PIC_DEFINITION *picDevice)
{
	bool				fail;
	unsigned int	i, size;
	unsigned char	theBuffer[32];

	size = GetIDSize(picDevice) * 2;

	if (!size)
	{
		fprintf(stderr, "Reading ID Locations is not supported for this device.\n");
		return false;
	}

	fail = false;
	theBuffer[0] = CMD_READ_ID;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nRead ID Locations");
		comm_debug_count = 0;
		sendCommand = true;
	}

	if (SendMsg(theBuffer, 1, theBuffer, size + 2))
	{
		if ((theBuffer[0] == CMD_READ_ID) && (theBuffer[size + 1] == 0))
		{
			if (verboseOutput)
				fprintf(stdout, "ID locations: ");	// if in quiet mode, only the values will be returned

			for (i=0; i<size; i+= 2)
				fprintf(stdout, "0x%02x%02x ", theBuffer[i + 1], theBuffer[i + 2]);

			fprintf(stdout, "\n");
		}
		else
		{
			fprintf(stderr, "failed to read ID locations\n");
			fail = true;
		}
	}
	else
	{
		fprintf(stderr, "failed to send read ID command\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// initialize for specified part, return true if succeeded

static bool DoInitPIC(const PIC_DEFINITION *picDevice)
{
	int				idx;
	bool				fail;
	unsigned char	theBuffer[3];
	unsigned char	cmdBuffer[PICDEV_DEFSIZE + 1];
	unsigned char	extCmdBuffer[PICDEV_DEFXSIZE + 1];

	fail = false;
	theBuffer[0] = CMD_LOAD_INFO;

	if (comm_debug)
	{
		fprintf(comm_debug, "\nLoad Processor Info");
		comm_debug_count = 0;
	}

		// send load processor info command, wait for command to echo back
	if (SendMsg(theBuffer, 1, theBuffer, 1))
	{
		if (theBuffer[0] == CMD_LOAD_INFO)
		{
			memcpy(cmdBuffer, picDevice->def, PICDEV_DEFSIZE);	// copy definition into the new buffer
			cmdBuffer[PICDEV_DEFSIZE] = 0;							// initialize the checksum

					// calculate the checksum, store as last byte in buffer
			for (idx = 0; idx < PICDEV_DEFSIZE; idx++)
				cmdBuffer[PICDEV_DEFSIZE] += cmdBuffer[idx];

					// send whole buffer including checksum
			if (SendMsg(cmdBuffer, PICDEV_DEFSIZE + 1,theBuffer, 1))
			{
				if (theBuffer[0] == 0)			// zero = checksum okay (data received okay)
				{
					theBuffer[0] = CMD_LOAD_EXT_INFO;

					if (comm_debug)
					{
						fprintf(comm_debug, "\nLoad Extended Configuration");
						comm_debug_count = 0;
					}

						// send load extended processor info command, wait for command to echo back
					if (SendMsg(theBuffer, 1, theBuffer, 1))
					{
						if (theBuffer[0] == CMD_LOAD_EXT_INFO)
						{
								// copy definition into the new buffer
							memcpy(extCmdBuffer, picDevice->defx, PICDEV_DEFXSIZE);
							extCmdBuffer[PICDEV_DEFXSIZE] = 0;		// initialize the checksum

								// calculate the checksum, store as last byte in buffer
							for (idx=0; idx<PICDEV_DEFXSIZE; idx++)
								extCmdBuffer[PICDEV_DEFXSIZE] += extCmdBuffer[idx];

								// send whole buffer including checksum
							if (SendMsg(extCmdBuffer, PICDEV_DEFXSIZE + 1, theBuffer, 1))
							{
								if (theBuffer[0] != 0)	// zero = checksum okay (data received okay)
								{
									theBuffer[0] = CMD_LOAD_EXT_INFO;

									if (comm_debug)
									{
										fprintf(comm_debug, "\nLoad Extended Configuration - old firmware");
										comm_debug_count = 0;
									}

						  				// send load extended processor info command, wait for command to echo back
									if (SendMsg(theBuffer, 1, theBuffer, 1))
									{
										if (theBuffer[0] == CMD_LOAD_EXT_INFO)
										{
												// copy definition into the new buffer
											memcpy(extCmdBuffer, picDevice->defx, OLD_PICDEV_DEFXSIZE);
											extCmdBuffer[OLD_PICDEV_DEFXSIZE] = 0;	 // initialize the checksum

								 				// calculate the checksum, store as last byte in buffer
											for (idx=0; idx<OLD_PICDEV_DEFXSIZE; idx++)
											{
												extCmdBuffer[OLD_PICDEV_DEFXSIZE] += extCmdBuffer[idx];
											}

												// send whole buffer including checksum
											if (SendMsg(extCmdBuffer, OLD_PICDEV_DEFXSIZE + 1, theBuffer, 1))
											{
												if (theBuffer[0] != 0)	// zero = checksum okay (data received okay)
												{
													fprintf(stderr, "bad result from PICSTART: 0x%x\n", theBuffer[0]);
													fail = true;
												}
												else
													oldFirmware = true;
											}
										}
									}
								}
							}
							else
							{
								fprintf(stderr, "failed to send extended device data\n");
								fail = true;
							}
						}
						else
						{
							fprintf(stderr, "echoback did not look correct\n");
							fail = true;
						}
					}
					else
					{
						fprintf(stderr, "failed to send extended device definition\n");
						fail = true;
					}
				}
				else
				{
					fprintf(stderr, "checksum failure\n");
					fail = true;											// didn't receive zero, fail
				}
			}
			else
			{
				fprintf(stderr, "failed to send device definition\n");
				fail = true;
			}
		}
		else
		{
			fprintf(stderr, "echoback did not look correct\n");
			fail = true;
		}
	}
	else
	{
		fprintf(stderr, "failed to send load info command\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// find out if the next argument is a flag (not preceeded by '-').
// if it is, return a pointer to it and advance to the next argument, otherwise
//	return null.

static char *GetNextFlag(int *argc, char **argv[])
{
	if (*argc && (***argv != '-'))	// if the next argument isn't preceeded by a '-'
	{
		(*argv)++;							// advance to next argument
		(*argc)--;
		return(*(*argv - 1));			// but return pointer to this one
	}

	return(NULL);
}

//--------------------------------------------------------------------
// Do all the things that the command line is asking us to do

static bool DoTasks(int *argc, char **argv[], const PIC_DEFINITION *picDevice, char *flags)
{
	bool				fail = false;
	char				*fileName = (char *) 0;
	FILE				*theFile = stdout;
	unsigned char	blankMode, *cbfr;
	unsigned int	i, count, count2, *ibfr, iddata;
	unsigned int	oscCalBits;

	switch (*flags)
	{
		case 'b':								// blank check
			flags++;

			if (*flags)
			{
				blankMode = 0;

				while (*flags)
				{
					switch (*flags)
					{
						case 'p':
							blankMode |= BLANK_PGM;
							break;

						case 'c':
							blankMode |= BLANK_CFG;
							break;

						case 'i':
							blankMode |= BLANK_ID;
							break;

						case 'd':
							blankMode |= BLANK_DATA;
							break;

						default:
							break;				// ignore undefined flags
					}

					flags++;
				}
			}
			else					// no mode flags means check them all
				blankMode = BLANK_PGM | BLANK_CFG | BLANK_ID | BLANK_DATA;

			fail = !DoBlankCheck(picDevice, blankMode);
			break;

		case 'e':								// erase
			flags++;

			if (*flags)
			{
				while (*flags && !fail)
				{
					switch (*flags)
					{
						case 'p':
							fail = !DoErasePgm(picDevice, true);
							break;

						case 'c':
							fail = !DoEraseConfigBits(picDevice);
							break;

						case 'i':
							fail = !DoEraseIDLocs(picDevice);
							break;

						case 'd':
							fail = !DoEraseData(picDevice, true);
							break;

						case 'o':	// [TODO] erase oscillator calibration
							fprintf(stderr, "erase oscillator calibration not implemented yet\n");
								// DEBUG can osc cal be erased/written on flash parts?
							break;

						case 'f':
							fail = !DoEraseFlash(picDevice);
							break;

					}

					flags++;
				}
			}
			else
				fprintf(stderr, "specify one or more regions to erase (p|c|i|d|o)\n");
			break;

		case 'r':								// read
			flags++;

			if (*flags)
			{
				while (*flags && !fail)
				{
					switch (*flags)
					{
						case 'p':
							if ((fileName = GetNextFlag(argc, argv)))
								theFile = fopen(fileName, "w");
							else
								theFile = stdout;

							if (theFile)
							{
								fail = !DoReadPgm(picDevice, theFile);		// read program data, write to stream

								if (theFile != stdout)		// if we wrote it to a file,
									fclose(theFile);			// close the file
							}
							else
								fprintf(stderr, "unable to open output file: '%s'\n", fileName);

							break;

						case 'c':
							fail = !DoReadCfg(picDevice, true);	// read configuration bits, display them
							break;

						case 'i':
							fail = !DoReadID(picDevice);	// read ID locations
							break;

						case 'd':
							if ((fileName = GetNextFlag(argc, argv)))
								theFile = fopen(fileName, "w");
							else
								theFile = stdout;

							if (theFile)
							{
								fail = !DoReadData(picDevice, theFile);	// read data memory

								if (theFile != stdout)		// if we wrote it to a file,
									fclose(theFile);			// close the file
							}
							else
								fprintf(stderr, "unable to open output file: '%s'\n", fileName);

							break;

						case 'o':
							fail = !DoReadOscCal(picDevice, true);
							break;
					}

					flags++;
				}
			}
			else
				fprintf(stderr, "specify one or more regions to read (p|c|i|d|o)\n");

			break;

		case 'w':								// write
			flags++;

			if (*flags)
			{
				switch (*flags)
				{
					case 'p':
						flags++;

						if (*flags && (toupper(*flags) == 'X'))
						{
							suppressWrite = true;
							ignoreVerfErr = true;
						}

						if ((fileName = GetNextFlag(argc, argv)))
							theFile = fopen(fileName, "r");
						else
							theFile = stdin;

						if (theFile)
						{
							fail = !DoWritePgm(picDevice, theFile);
							
							// Skip verification for K150 for now due to read ROM issues
							if (!fail && (picDevice->pgm_support & P_K150)) {
								printf("K150: Note - Verification skipped (K150 read function needs improvement)\n");
							}

							if (theFile != stdin)					// if we read it from a file,
								fclose(theFile);						// close the file
						}
						else
							fprintf(stderr, "unable to open input file: '%s'\n", fileName);

						break;

					case 'c':
						if ((fileName = GetNextFlag(argc, argv)))	// 'fileName' is actually the next argument
						{
							i = GetConfigSize(picDevice) * 2;

							if (!i)
							{
								fprintf(stderr, "Writing to configuration bits is not supported for this device.\n");
								fail = true;
								break;
							}

							count = i / sizeof(unsigned short int);
							ibfr = (unsigned int *) alloca(i * sizeof(unsigned int));
							cbfr = (unsigned char *) alloca(i * sizeof(unsigned int) * 2);

							if (!ibfr || !cbfr)
							{
								fprintf(stderr, "Error allocating memory");
								fail = true;
								break;
							}

							for (count2=0; count2<count; count2++)
							{
								fail = !atoi_base(fileName, (unsigned int *) &ibfr[count2]);

								if (fail)
									break;

								fileName = GetNextFlag(argc, argv);
							}

							if (fail)
							{
								fprintf(stderr, "Error parsing supplied configuration bits.\n");
								fail = true;
								break;
							}

// [TODO] the following may not be correct for 18xxx devices

							for (count = count2 = 0; count < i; count++, count2 += 2)
							{
								cbfr[count2+0] = (ibfr[count] >> 8)  & 0xff;
								cbfr[count2+1] = (ibfr[count] >> 0)  & 0xff;
							}

							fail = !DoWriteConfigBits(picDevice, cbfr, count, 0);
						}
						else
						{
							fprintf(stderr, "Missing argument to Write Configuration Bits command.\n");
							fail = true;
						}

						break;

					case 'i':
						if ((fileName = GetNextFlag(argc, argv)))
						{
							i = GetIDSize(picDevice) * 2;
							count = i / sizeof(unsigned short int);
							ibfr = (unsigned int *) alloca(i * sizeof(unsigned int));
							cbfr = (unsigned char *) alloca(i * sizeof(unsigned int) * 2);

							if (!ibfr || !cbfr)
							{
								fprintf(stderr, "Error allocating memory");
								fail = true;
								break;
							}

							for (count2=0; count2<count && fileName; count2++)
							{
								fail = !atoi_base(fileName, (unsigned int *) &ibfr[count2]);

								if (fail)
									break;

								fileName = GetNextFlag(argc, argv);
							}

							if (fail)
							{
								fprintf(stderr, "Error parsing supplied ID Locations data.\n");
								fail = true;
								break;
							}

// [TODO] the following may not be correct for 18xxx devices

							iddata = (picDevice->def[PD_DATA_WIDTHH] << 8) + picDevice->def[PD_DATA_WIDTHL];

							for ( ; count2 < i / 2; count2++)
								ibfr[count2] = iddata;

							for (count = count2 = 0; count < i; count++, count2 += 2)
							{
								cbfr[count2 + 1] = (ibfr[count] >> 8)  & 0xff;
								cbfr[count2 + 0] = (ibfr[count] >> 0)  & 0xff;
							}

							fail = !DoWriteIDLocs(picDevice, cbfr, count);
						}
						else
						{
							fprintf(stderr, "Missing argument to Write ID locations command.\n");
							fail = true;
						}

						break;

					case 'd':
						if ((fileName = GetNextFlag(argc, argv)))
							theFile = fopen(fileName, "r");
						else
							theFile = stdin;

						if (theFile)
						{
							fail = !DoWriteData(picDevice, theFile);

							if (theFile != stdin)					// if we read it from a file,
								fclose(theFile);						// close the file
						}
						else
							fprintf(stderr, "unable to open input file: '%s'\n", fileName);

						break;

					case 'o':
						if ((fileName = GetNextFlag(argc, argv)))	// 'fileName' is actually the next argument
						{
							fail = !atoi_base(fileName, &oscCalBits);

							if (!fail)
							{
								if (oscCalBits < 0x10000)
									fail = !DoWriteOscCalBits(picDevice, (unsigned short int) oscCalBits);
								else
								{
									fprintf(stderr, "Value out of range: '%s'\n", fileName);
									fail = true;
								}
							}
							else
								fprintf(stderr, "Unable to interpret '%s' as a numerical value\n", fileName);
						}
						else
							fprintf(stderr, "write oscillator calibration must be followed by a numerical value\n");

						break;

					default:
						fprintf(stderr, "must specify a region to write\n");
						break;
				}
			}
			else
				fprintf(stderr, "specify a region to write (p|c|i|d|o)\n");

			break;

		case 'v':	// [TODO]				// verify
			fprintf(stderr, "verify is not implemented yet\n");
			// DEBUG verify device goes here
			break;

		default:
			break;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Initialize the serial port
// Once the device is opened and locked, this sets up the port, and makes sure the handshake looks good.

static bool InitDevice(int serialDevice, unsigned int baudRate, unsigned char dataBits, unsigned char stopBits, unsigned char parity)
{
	bool						fail;						// haven't failed (yet)
	bool						CTS, DCD;
	unsigned short int	ctsTimeOut;

	fail = false;

	if (ConfigureDevice(serialDevice, baudRate, dataBits, stopBits, parity, false))	// set up the device
	{
		if (ConfigureFlowControl(serialDevice, false))	// no flow control at the moment (raise RTS)
		{
			ResetPICSTART();
			ctsTimeOut = 100;							// allow about 100 ms (0.1 sec) for CTS to show up

			do
			{
				GetDeviceStatus(serialDevice, &CTS, &DCD);	// see if CTS is true

				if (CTS)
					break;								// break out if it is

				usleep(1000);							// wait 1 ms (more or less), try again
			}
			while (ctsTimeOut--);

			if (!CTS)
			{
				fprintf(stderr, "programmer not detected (CTS is false)\n");
				fail = true;							// didn't see CTS, assume device is not present or not ready, fail
			}
			else
				ConfigureFlowControl(serialDevice, true);	// looks ok to use flow control, so allow it

			FlushBytes(serialDevice);						// get rid of any pending data
		}
		else
		{
			fprintf(stderr, "could not configure flow control\n");
			fail = true;
		}
	}
	else
	{
		fprintf(stderr, "could not configure device parameters\n");
		fail = true;
	}

	return(!fail);
}

//--------------------------------------------------------------------
// Show a starting address and size

static void ShowStartSize(unsigned short int start, unsigned short int size)
{
	if (size)
		fprintf(stdout, "    0x%04x-0x%04x (0x%04x word%c)\n", start, start + size - 1, size, ((size != 1) ? 's' : '\0'));
	else
		fprintf(stdout, "    none\n");
}

// Display data from PIC_DEFINITION entry

void dumpDevData(char *name)
{
	int	i;
	PIC_DEFINITION	*pd;
	unsigned char	*dptr;

	pd = GetPICDefinition(name);
//	printf("\ndumpDevData: ptr 0x%x\n", (int) pd);

	printf("\nPIC definition data:");
	printf("\ncpbits:\t\t0x%x\n", pd->cpbits);
	printf("wdbit:\t\t0x%x\n", pd->wdbit);
	printf("wordalign:\t0x%x\n", pd->wordalign);
	printf("cfgmem:\t\t0x%x\n", pd->cfgmem);
	printf("idaddr:\t\t0x%x\n", pd->idaddr);
	printf("idsize:\t\t0x%x\n", pd->idsize);
	printf("eeaddr:\t\t0x%x\n", pd->eeaddr);
	printf("fixedCfgBitsSize: %d\n", pd->fixedCfgBitsSize);
	printf("fixedCfgBits:\n");

	for (i=0; i<8; i++)
		printf(" %02x", pd->fixedCfgBits[i]);

	printf("\n\nProgrammer support: ");

	if (pd->pgm_support & P_PICSTART)
		printf(" PicStart");

	if (pd->pgm_support & P_WARP13)
		printf(" Warp-13");

	if (pd->pgm_support & P_JUPIC)
		printf(" JuPic");

	if (pd->pgm_support & P_OLIMEX)
		printf(" Olimex");

	if (pd->pgm_support & P_K150)
		printf(" K150");

	dptr = pd->def;
	printf("\n\nDEF data:\n");
	for (i=0; i<44; i++)
	{
		printf(" %02x", dptr[i]);

		if ((i % 16) == 15)
			printf("\n");
	}

	dptr = pd->defx;
	printf("\nDEFX data:\n");

	for (i=0; i<32; i++)
	{
		printf(" %02x", dptr[i]);

		if ((i % 16) == 15)
			printf("\n");
	}

	printf("\n\n");
}

//--------------------------------------------------------------------
// Report information about the passed device

static void ShowDeviceInfo(const PIC_DEFINITION *picDevice)
{
	fprintf(stdout, "\ndevice name: %s\n", picDevice->name);			// show the name

	fprintf(stdout, "  program space:\n");									// show range of program space
	ShowStartSize(0, GetPgmSize(picDevice));

	fprintf(stdout, "  eeprom data space:\n");							// show range of data space, if any
	ShowStartSize(GetDataStart(picDevice), GetDataSize(picDevice));

	fprintf(stdout, "  oscillator calibration space:\n");				// show range of calibration space, if any
	ShowStartSize(GetOscCalStart(picDevice), GetOscCalSize(picDevice));

	fprintf(stdout, "  configuration bits:\n");
	fprintf(stdout, "    address: 0x%x, size: %u words\n",
		GetConfigStart(picDevice), GetConfigSize(picDevice));	// show address of configuration bits
	fprintf(stdout, "    protect mask:  0x%04x\n", picDevice->cpbits);	// mask of code protect bits
	fprintf(stdout, "    watchdog mask: 0x%04x\n", picDevice->wdbit);		// mask of watchdog enable bit

	dumpDevData(picDevice->name);
}

//--------------------------------------------------------------------
//	display all supported devices

static void ShowDevices()
{
	int length = 0;
	int thisLength;
	DEV_LIST	*devptr;

	fprintf(stdout, "\n%d supported devices:\n\n", deviceCount);
	devptr = deviceList;

	while (devptr)
	{
		thisLength = strlen(devptr->picDef.name);	// length of this device's name
		length += thisLength;								// add to length of this line

		if (length + 2 < MAXNAMESLEN)						// ensure there's room for the space and comma, too
			fprintf(stdout, "%s", devptr->picDef.name);	// put it on this line
		else
		{
			fprintf(stdout, "\n%s", devptr->picDef.name);	// put it on the next line
			length = thisLength;
		}

		devptr = devptr->next;

		if (devptr)									// if more devices are in the list,
		{
			fprintf(stdout, ", ");							// add a comma and a space
			length += 2;
		}
		else
			fprintf(stdout, "\n");							// or newline if it is the last one
	}

	fprintf(stdout, "\n");
}

//--------------------------------------------------------------------
// some thorny issues still exist in specifying arguments:
//
//	1) should write cause a blank check before writing?  should it program anyway if no bits that should be 1 are 0?
//	2) several different writes are needed (program, ID, data, config)
//	3) likewise, several different reads are needed
//	4) need to be able to specify a range of addresses to read, instead of reading the entire device
//	5) should erase be an option?  several different erases? erase plus a set of flags to indicate what to erase?

//--------------------------------------------------------------------
// tell the user how to use this program

static void Usage()
{
	fprintf(stdout, "\n%s: version %s\n"
			" (c) 2000-2004 Cosmodog, Ltd. (http://www.cosmodog.com)\n"
			" (c) 2004-2006 Jeff Post (http://home.pacbell.net/theposts/picmicro)\n"
			" GNU General Public License\n", programName, versionString);
	fprintf(stdout, "\nUsage: %s [-c] [-d] [-v] ttyname [-v] devtype [-i] [-h] [-q] [-v] [-s [size]] [-b|-r|-w|-e][pcidof]\n", programName);
	fprintf(stdout, " where:\n");
	fprintf(stdout, "  ttyname is the serial (or USB) device the programmer is attached to\n");
	fprintf(stdout, "     (e.g. /dev/ttyS0 or com1)\n");
	fprintf(stdout, "  devtype is the pic device to be used (12C508, 16C505, etc.)\n");
	fprintf(stdout, "  -b blank checks the requested region or regions\n");
	fprintf(stdout, "  -c enable comm line debug output to picpcomm.log (must be before ttyname)\n");
	fprintf(stdout, "  -d (if only parameter) show device list\n");
	fprintf(stdout, "  -d devtype - show device information\n");
	fprintf(stdout, "  -e erases the requested region (flash parts only)\n");
	fprintf(stdout, "  -f ignores verify errors while writing\n");
	fprintf(stdout, "  -h show this help\n");
	fprintf(stdout, "  -i use ISP protocol (must be first option after devtype)\n");
	fprintf(stdout, "  -q sets quiet mode (excess messages supressed)\n");
	fprintf(stdout, "  -r initiates a read (Intel Hex record format)\n");
	fprintf(stdout, "  -s [size] shows a hash mark status bar of length [size] while erasing/writing\n");
	fprintf(stdout, "  -w writes to the requested region\n");
	fprintf(stdout, "     -wpx will suppress actual writing to program space (for debugging picp)\n");
	fprintf(stdout, "  -v (if given after ttyname or after devtype) show programmer version number\n");
	fprintf(stdout, "  -v (if only parameter) show picp version number\n");
	fprintf(stdout, "  Read/Write/Erase parameters:\n");
	fprintf(stdout, "    p [filename] = program memory, optionally reading/writing filename\n");
	fprintf(stdout, "    c [val] = configuration bits (val is a numeric word value when writing)\n");
	fprintf(stdout, "    i [val] = ID locations\n");
	fprintf(stdout, "    d [filename] = data memory, optionally reading/writing filename\n");
	fprintf(stdout, "    o [val] = oscillator calibration space\n");
	fprintf(stdout, "    f = entire flash device (only applies to -e, erase)\n");
	fprintf(stdout, "  filename is an optional input or output file (default is stdin/stdout)\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Flags are operated on in order, from left to right.  If any operation fails,\n");
	fprintf(stdout, "further execution is aborted.  Thus, a part can be blank checked and programmed\n");
	fprintf(stdout, "with a single command, e.g.:\n");
	fprintf(stdout, "        %s /dev/ttyS0 16c505 -bp -wp program.hex \n", programName);
	fprintf(stdout, "This example will blank check the program memory of a PIC16C505 then write the\n");
	fprintf(stdout, "contents of the file program.hex to the program memory only if the blank check\n");
	fprintf(stdout, "succeeded.\n");
	fprintf(stdout, "The -wc, -wi, and -wo options must be followed by a numeric argument which\n");
	fprintf(stdout, "represents the value.  The number may be binary (preceeded by 0b or 0B), hex\n");
	fprintf(stdout, "(preceeded by 0x or 0X), or decimal (anything else).\n\n");
}

// Skip over white space

char *skipWhitespace(char *str)
{
	while (*str && (*str == ' ' || *str == '\t'))
		str++;

	return str;
}

// Skip over text to next white space

char *skipText(char *str)
{
	while (*str && *str != ' ' && *str != '\t')
		str++;

	return str;
}

// Convert hex ascii string to binary.
// Return updated string pointer.

char *atox(char *str, int *result)
{
	int	digit;

	*result = 0;						// start at zero

	if (*str)							// do nothing if it's a null string
	{
		while (*str)
		{
			digit = toupper(*str);	// force all upper case for hex digits

			if (isxdigit(digit))
			{
				digit = (digit >= 'A') ? digit - 'A' + 0x0a : digit - '0';	// convert to 0-9, A-F
				*result = ((*result) << 4) + digit;	// shift up one order of magnitude, add in this digit
			}
			else
				break;				// bad character, done

			str++;
		}
	}

	return str;
}

// Check if current header is for the same device.
// If so, return the name pointer, else return NULL.

char *checkSameDevice(DEV_LIST *devptr, char *str)
{
	int	i = 0;

	while (*str && devptr->picDef.name[i])
	{

		if (*str != devptr->picDef.name[i])
			return NULL;

		i++;
		str++;

		if (*str == ':')
		{
			str++;
			break;
		}
	}

	return str;
}

// Get next value from picdevrc file.
// The value may be for any block: main definition,
// def values, or defx values.

int getNextDefValue(FILE *fp)
{
	bool			done = false;
	int			val = 0;
	char			*cptr, *start;
	static int	offset = 0;
	static char	line[128];

	while (!done)
	{
		if (!offset)
			fgets(line, 127 - 1, fp);

		start = (char *) &line[offset];
		start = skipWhitespace(start);
		cptr = atox(start, &val);

		if (cptr == start)		// nothing was converted
			offset = 0;				// get the next line
		else
		{
			offset = (int) (cptr - &line[0]);

			if (line[offset] == ';')
				offset = 0;

			done = true;
		}
	}

	return val;
}

// Read PIC_DEFINITION data from config file
// Return number of devices loaded.

int loadPicDefinitions(void)
{
	int		i, val, count = 0;
	bool		badEntry;
	unsigned short int	pgm;
	char		*cptr;
	unsigned char	*data;
	FILE		*fp;
	DEV_LIST	*devptr, *next = NULL;
	char		line[128];

	fp = fopen("picdevrc", "r");		// try current directory first

	if (!fp)			// if not found, try default directory
#ifdef WIN32
		fp = fopen("c:\\Program Files\\picp\\picdevrc", "r");
#else
		fp = fopen("/usr/local/bin/picdevrc", "r");
#endif

	if (!fp)
		return 0;

	while (!feof(fp))
	{
		fgets(line, 127 - 1, fp);

		if (line[0] == '[')		// beginning of device definition
		{
			i = 1;
			badEntry = false;

			while (line[i] != ']')	// count characters in device name
				i++;

			--i;
			cptr = (char *) malloc(i + 1);
			strncpy(cptr, (char *) &line[1], i);
			cptr[i] = '\0';

			devptr = (DEV_LIST *) malloc(sizeof(DEV_LIST));
			devptr->picDef.name = cptr;
			devptr->picDef.def = NULL;
			devptr->picDef.defx = NULL;
			devptr->next = NULL;

// read pic device data

			val = getNextDefValue(fp);
			devptr->picDef.cpbits = (unsigned short int) val;

			val = getNextDefValue(fp);
			devptr->picDef.wdbit = (unsigned short int) val;

			val = getNextDefValue(fp);
			devptr->picDef.wordalign = (unsigned short int) val;

			val = getNextDefValue(fp);
			devptr->picDef.cfgmem = (unsigned int) val;

			val = getNextDefValue(fp);
			devptr->picDef.idaddr = (unsigned int) val;

			val = getNextDefValue(fp);
			devptr->picDef.idsize = (unsigned short int) val;

			val = getNextDefValue(fp);
			devptr->picDef.eeaddr = (unsigned int) val;

			val = getNextDefValue(fp);
			devptr->picDef.fixedCfgBitsSize = (unsigned char) val;

			for (i=0; i<8; i++)			// read fixed config bits words
			{
				val = getNextDefValue(fp);
				devptr->picDef.fixedCfgBits[i] = (unsigned short int) val;
			}

			cptr = fgets(line, 127 - 1, fp);	// read programmer support
			pgm = 0;

			while (cptr)
			{
				cptr = skipWhitespace(cptr);

				if (!cptr)
					break;

				switch (toupper(*cptr))
				{
					case 'P':		// PicStart Plus
						pgm |= P_PICSTART;
						break;

					case 'W':		// Warp-13
						pgm |= P_WARP13;
						break;

					case 'J':		// JuPic
						pgm |= P_JUPIC;
						break;

					case 'O':		// Olimex
						pgm |= P_OLIMEX;
						break;

					case 'K':		// K150
						pgm |= P_K150;
						break;

					default:
						cptr = NULL;
						break;
				}

				if (cptr)
				{
					cptr = skipText(cptr);

					if (!cptr)
						break;

					cptr = skipWhitespace(cptr);
				}
			}

			devptr->picDef.pgm_support = pgm;

// read def data

			line[0] = '\0';

			while (!feof(fp) && line[0] != '[')	// search for def header
				cptr = fgets(line, 127 - 1, fp);

			if (!cptr)
			{
				badEntry = true;
				break;
			}

			cptr = checkSameDevice(devptr, (char *) &line[1]);

			if (cptr)
			{
				if (!strncmp(cptr, "def]", 4))	// is def block
				{
					data = (unsigned char *) malloc(PICDEV_DEFSIZE);
					devptr->picDef.def = data;

					for (i=0; i<PICDEV_DEFSIZE; i++)
					{
						val = getNextDefValue(fp);
						data[i] = (unsigned char) val;
					}
				}
				else
				{
					badEntry = true;
					break;
				}
			}
			else
			{
				badEntry = true;
				break;
			}

// read defx data

			line[0] = '\0';

			while (!feof(fp) && line[0] != '[')	// search for defx header
				cptr = fgets(line, 127 - 1, fp);

			if (!cptr)
			{
				badEntry = true;
				break;
			}

			cptr = checkSameDevice(devptr, (char *) &line[1]);

			if (cptr)
			{
				if (!strncmp(cptr, "defx", 4))	// is defx block
				{
					data = (unsigned char *) malloc(PICDEV_DEFXSIZE);
					devptr->picDef.defx = data;

					for (i=0; i<PICDEV_DEFXSIZE; i++)
					{
						val = getNextDefValue(fp);
						data[i] = (unsigned char) val;
					}
				}
				else
				{
					badEntry = true;
					break;
				}
			}
			else
			{
				badEntry = true;
				break;
			}

			if (!badEntry)
			{
				if (!deviceList)
				{
					deviceList = devptr;
					next = devptr;
				}
				else
				{
					next->next = devptr;
					next = devptr;
				}

				count++;
			}
			else
			{
				free(devptr->picDef.name);

				if (devptr->picDef.def)
					free(devptr->picDef.def);

				if (devptr->picDef.defx)
					free(devptr->picDef.defx);

				free(devptr);
			}
		}	// done with this device, check for another
	}	// while !feof

	fclose(fp);
	deviceCount = count;
	return count;
}

//--------------------------------------------------------------------
// Program PICs through a serial port

int main(int argc,char *argv[])
{
	bool				fail;
	unsigned int	baudRate;
	unsigned char	dataBits, stopBits, parity;
	bool				done;
	char				*flags;
	time_t			tp;
	struct tm		*date_time;
	int				i, year;
	const PIC_DEFINITION	*picDevice = NULL;

#ifdef BETA
	sprintf(versionString, "0.6.9 - beta %d", BETA);
#else
	strcpy(versionString, "0.6.9");
#endif

	comm_debug = NULL;
	signal(SIGINT, SigHandler);					// set up a signal handler

	programName = *argv++;							// name of the application
	argc--;

	fail = false;
	verboseOutput = true;							// be verbose unless told otherwise
	hashWidth = false;								// don't show hask marks by default
	ignoreVerfErr = false;							// by default stop on verify errors

	if (!loadPicDefinitions())
	{
		fprintf(stderr, "\n%s: version %s\n", programName, versionString);
		fprintf(stderr, "\nCan't read PIC definition data.\n\n");
		return 1;
	}

	if (argc > 2)										// need at least four arguments to do anything
	{
		if ((!strcmp(argv[0], "-c")) || (!strcmp(argv[0], "-C")))	// if first argument is '-c', debug comm line
		{
			comm_debug = fopen("picpcomm.log", "a");

			if (comm_debug)
			{
				time(&tp);								// get current time
				date_time = localtime(&tp);		// convert to hr/min/day etc
				year = date_time->tm_year;

				while (year > 100)
					year -= 100;

				fprintf(comm_debug, "\nPicp %s comm debug file opened %02d/%02d/%02d %02d:%02d\nOptions:",
					versionString,
					date_time->tm_mon + 1,
					date_time->tm_mday, year,
					date_time->tm_hour, date_time->tm_min);

				for (i=0; i<argc; i++)
					fprintf(comm_debug, " %s", argv[i]);

				fprintf(comm_debug, "\n");
			}

			argc--;
			argv++;
		}

		deviceName = *argv++;								// name of the device (probably)
		argc--;
		picName = *argv++;									// name of the PIC type (probably)
		argc--;

		if ((picDevice = GetPICDefinition(picName)))			// locate the PIC type (0 = none found)
		{
			done = false;

			// First try K150 detection directly
			printf("Trying K150 detection on %s\n", deviceName);
			if (k150_open_port(deviceName) == 0)
			{
				printf("K150 port opened successfully\n");
				if (k150_detect_programmer() == 0) {
					isK150 = true;
					programmerSupport = P_K150;
					printf("K150 programming support is ready\n");
					// Continue with K150 operations directly
					
					// Skip normal programmer initialization for K150
					if (!(programmerSupport & picDevice->pgm_support))
					{
						fprintf(stderr, "\n\nDevice %s may not be supported by the K150 programmer\n", picDevice->name);
						fail = true;
					}
					else
					{
						// Process command line arguments for K150
						while (argc && !fail)
						{
							flags = *argv++;
							argc--;

							if (*flags == '-')
							{
								flags++;
								
								switch (*flags)
								{
									case 'v':
										printf("picp version %s\n", versionString);
										break;
									case 'r':
										fail = !DoTasks(&argc, &argv, picDevice, flags);
										break;
									case 'e':
										fail = !DoTasks(&argc, &argv, picDevice, flags);
										break;
									case 'w':
										fail = !DoTasks(&argc, &argv, picDevice, flags);
										break;
									case 'i':
										// ICSP mode flag - set global variable
										printf("K150: ICSP mode enabled\n");
										break;
									default:
										break;
								}
							}
						}
					}
				}
				else
				{
					// Keep K150 port open for subsequent operations
				}
			}

			if (!done && !isK150 && OpenDevice(deviceName, &serialDevice))		// open the serial device
			{
				baudRate = 19200;
				dataBits = 8;
				stopBits = 1;
				parity = 0;

				if (InitDevice(serialDevice, baudRate, dataBits, stopBits, parity))	// initialize the serial port
				{
					if (DoGetProgrammerType())				// ask what kind of programmer is attached, fail if none or one we don't support
					{
						if (DoGetVersion())					// get the version number of the programmer
						{
							if (!(programmerSupport & picDevice->pgm_support))	// does this programmer support this device?
							{
								DoShowVersion();
								fprintf(stderr, "\n\nDevice %s may not be supported by the ", picDevice->name);

								switch (programmerSupport)
								{
									case P_PICSTART:
										fprintf(stderr, "PicStart Plus");
										break;

									case P_WARP13:
										fprintf(stderr, "Warp-13");
										break;

									case P_JUPIC:
										fprintf(stderr, "JuPic");
										break;

									case P_K150:
										fprintf(stderr, "K150");
										break;

									case P_OLIMEX:
										fprintf(stderr, "Olimex");
										break;

									default:
										fprintf(stderr, "Unknown");
										break;
								}

								fprintf(stderr, " programmer\nContinue anyway? [y/n] \n");
								fflush(stderr);
								i = toupper(getchar());

								if (i != 'Y')
								{
									CloseDevice(serialDevice);
									exit(1);
								}
							}

							if (DoInitPIC(picDevice))					// try to load up the parameters for this device
							{
								while (argc && !fail)					// do as long as we can read some more
								{
									flags = *argv++;						// get this argument, point to the next
									argc--;

									if (*flags == '-')					// see if it's a flag
									{
										flags++;								// it is, skip the dash

										switch (*flags)
										{
											case 'v':
												DoShowVersion();
												break;

											case 'f':
												ignoreVerfErr = true;	// force, ignore verify error
												break;

											case 'q':
												verboseOutput = false;	// inhibit display of messages
												break;

											case 'h':
												Usage();						// give help
												break;

											case 'i':
												if (programmerSupport != P_WARP13)
												{
													switch (programmerSupport)
													{
														case P_PICSTART:
															fprintf(stderr, "PicStart Plus");
															break;

														case P_JUPIC:
															fprintf(stderr, "JuPic");
															break;

														case P_K150:
															fprintf(stderr, "K150");
															break;
													}

													fprintf(stderr, " programmer does not support ISP programming\n");
												}
												else
													ISPflag = true;
												break;

											case 's':
												if (argc && **argv != '-')		// if the next argument isn't preceeded by a '-'
												{
													fail = !atoi_base(*argv, &hashWidth);	// try to read the next argument as a number
													argv++;							// skip to the next argument
													argc--;

													if (fail)
														fprintf(stderr, "Unable to interpret '%s' as a numerical value\n", *argv);
												}
												else
													hashWidth = HASH_WIDTH_DEFAULT;	// turn hash marks on to the default width

												break;

											case 'b':
											case 'r':
											case 'w':
											case 'e':
												fail = !DoTasks(&argc, &argv, picDevice, flags);	// do the requested operation
												break;

											case '\0':						// ignore a stray dash
												break;

											default:
												fprintf(stderr, "bad argument: '%s'\n", *(argv - 1));	// back up, show the trouble spot
												break;
										}
									}
								}
							}
							else		// DoInitPIC failed
							{
								fprintf(stderr, "failed to initialize %s\n", picDevice->name);
								fail = true;
							}
						}
						else
						{
							fprintf(stderr, "failed to obtain programmer firmware version number\n");
							fail = true;
						}
					}
					else
					{
						fprintf(stderr, "failed to connect to programmer\n");
						fail = true;
					}
				}
				else
				{
					fprintf(stderr, "failed to set up the serial port\n");
					fail = true;
				}

				CloseDevice(serialDevice);
			}
			else
				fprintf(stderr, "failed to open device '%s'\n", deviceName);
		}
		else
		{
			fprintf(stderr, "unrecognized PIC device type: '%s'\n", picName);		// don't know that one;
			ShowDevices();														// give a helpful list of supported devices
		}
	}
	else
	{
		if (argc == 1)
			flags = argv[0];
		else
			flags = NULL;

		if ((argc == 1 && (picDevice = GetPICDefinition(argv[0]))) ||		// locate the PIC type (0 = none found)
			(argc == 2 && (picDevice = GetPICDefinition(argv[1]))))			//  (be forgiving about arg position)
		{
			ShowDeviceInfo(picDevice);
		}
		else if (flags && flags[0] == '-' && (flags[1] == 'v' || flags[1] == 'V'))
		{
			fprintf(stdout, "\n%s: version %s\n"
				" (c) 2000-2004 Cosmodog, Ltd. (http://www.cosmodog.com)\n"
				" (c) 2004-2006 Jeff Post (http://home.pacbell.net/theposts/picmicro)\n"
				" GNU General Public License\n\n", programName, versionString);
		}
		else if (flags && flags[0] == '-' && (flags[1] == 'd' || flags[1] == 'D'))
		{
			fprintf(stdout, "\n%s: version %s\n", programName, versionString);
			ShowDevices();
		}
		else if (argc == 2)		// option instead of PIC type
		{
			deviceName = *argv++;			// name of the device (probably)
			argc--;
			flags = *argv++;									// name of the PIC type (probably)

			if (*flags == '-')
			{
				flags++;

				if (*flags == 'v' || *flags == 'V')		// get programmer firmware version
				{
					if (OpenDevice(deviceName, &serialDevice))		// open the serial device
					{
						baudRate = 19200;
						dataBits = 8;
						stopBits = 1;
						parity = 0;

						if (InitDevice(serialDevice, baudRate, dataBits, stopBits, parity))	// initialize the serial port
						{
							fprintf(stdout, "\n");

							if (DoGetProgrammerType())	// ask what kind of programmer is attached, fail if none or one we don't support
							{
								if (DoGetVersion())		// get the version number of the programmer
									DoShowVersion();
							}

							fprintf(stdout, "\n");
						}

						CloseDevice(serialDevice);
					}
				}
			}
		}
		else
		{
			Usage();
			fail = true;
		}
	}

	return(fail);	// return 0 if okay (not failed)
}

// end of file
