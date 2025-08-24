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
// Copyright (c) 2004 Jeffery L. Post
//
// Please send bug reports for version 0.6.0 or higher to
// j_post <AT> pacbell <DOT> net.
//
//-----------------------------------------------------------------------------

// serial.c
// Handle opening and conditioning of the serial port
// These functions isolate all communications with the
// serial device

#include	<stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>

#ifdef WIN32
#include	<windows.h>
#define	false	FALSE
#define	true	TRUE
#else
#include <sys/ioctl.h>
#include <termios.h>
#include	<strings.h>
#endif

#include <fcntl.h>
#include <unistd.h>

#include	"serial.h"

#define MIN_CHARS		0		// DEBUG something is amiss with this, if VTIME is non-zero we get EAGAIN returned instead of zero (and no delay)
#define CHAR_TIMEOUT	0		// character timeout (read fails and returns if this much time passes without a character) in 1/10's sec

extern FILE	*comm_debug;
extern int	comm_debug_count;
extern bool	sendCommand;
extern bool	suppressWrite;

// See if there is unread data waiting on theDevice.
// This is used to poll theDevice without reading any characters
// from it.
// As soon as there is data, or timeOut expires, return.
// NOTE: timeOut is measured in microseconds
// if timeOut is 0, this will return the status immediately
bool ByteWaiting(int theDevice, unsigned int timeOut)
{
#ifndef WIN32
	fd_set	readSet;
	struct timeval	timeVal;

	FD_ZERO(&readSet);						// clear the set
	FD_SET(theDevice, &readSet);			// add this descriptor to the set
	timeVal.tv_sec = timeOut / 1000000;	// set up the timeout waiting for one to come ready
	timeVal.tv_usec = timeOut % 1000000;

	if (select(FD_SETSIZE, &readSet, NULL, NULL, &timeVal) == 1)	// if our descriptor is ready, then report it
		return(true);

	return(false);
#else
	COMSTAT comState;
	DWORD errors;

	ClearCommError ((HANDLE) theDevice, &errors, &comState);
	return comState.cbInQue > 0;
#endif
}

// Attempt to read at least one byte from theDevice before timeOut.
// once any byte is seen, attempt to get any more which are pending
// up to maxBytes
// If timeOut occurs, return 0
unsigned int ReadBytes(int theDevice, unsigned char *theBytes, unsigned int maxBytes, unsigned int timeOut)
{
#ifndef WIN32
	unsigned int	i, numRead = 0;
#else
	HANDLE			hCom = (HANDLE) theDevice;
	DWORD				numRead, i;
	COMMTIMEOUTS	cto;
#endif

#ifdef WIN32
	GetCommTimeouts(hCom, &cto );

	cto.ReadTotalTimeoutConstant = timeOut / 1000;
	SetCommTimeouts(hCom, &cto);
	ReadFile(hCom, theBytes, maxBytes, &numRead, NULL);
#else
	if (ByteWaiting(theDevice, timeOut))
	{
		numRead = read(theDevice, theBytes, maxBytes);
#endif
		if (numRead > 0)		// get waiting bytes
		{
			if (comm_debug)
			{
				for (i=0; i<numRead; i++)
				{
					fprintf(comm_debug, " I-0x%02x", theBytes[i]);

					if (sendCommand)
					{
						fprintf(comm_debug, "\n");
						sendCommand = false;
						comm_debug_count = 0;
					}
					else
						comm_debug_count++;

					if (comm_debug_count >= 8)
					{
						comm_debug_count = 0;
						fprintf(comm_debug, "\n");
					}
				}
			}
		}
#ifndef WIN32
		else
			numRead = 0;
	}
#endif

	return((int) numRead);
}

// Enhanced ReadBytes with retry mechanism for K150 operations
// Attempts multiple reads with delays to handle timing-sensitive hardware
unsigned int ReadBytesWithRetry(int theDevice, unsigned char *theBytes, unsigned int maxBytes, unsigned int timeOut, int maxRetries)
{
#ifndef WIN32
	unsigned int numRead = 0;
	int retry_count = 0;
	struct termios tio;
	
	// Full buffer flush before reading
	tcgetattr(theDevice, &tio);
	tcflush(theDevice, TCIOFLUSH);  // Input + output flush
	tcsetattr(theDevice, TCSANOW, &tio);
	
	while (retry_count < maxRetries && numRead == 0) {
		if (ByteWaiting(theDevice, timeOut)) {
			numRead = read(theDevice, theBytes, maxBytes);
			if (numRead > 0) {
				// Debug output for successful reads
				if (comm_debug) {
					for (unsigned int i = 0; i < numRead; i++) {
						fprintf(comm_debug, " I-0x%02x", theBytes[i]);
						if (sendCommand) {
							fprintf(comm_debug, "\n");
							sendCommand = false;
							comm_debug_count = 0;
						} else {
							comm_debug_count++;
						}
						if (comm_debug_count >= 8) {
							comm_debug_count = 0;
							fprintf(comm_debug, "\n");
						}
					}
				}
				break;  // Success, exit retry loop
			} else if (numRead < 0) {
				fprintf(stderr, "K150: Serial read error: %s\n", strerror(errno));
			}
		}
		
		retry_count++;
		if (retry_count < maxRetries) {
			usleep(50000);  // 50ms delay between retries
		}
	}
	
	if (numRead == 0) {
		fprintf(stderr, "K150: Read timeout after %d retries, wanted %d bytes, got %d\n", maxRetries, maxBytes, numRead);
	}
	
	return numRead;
#else
	// Windows implementation with retry
	HANDLE hCom = (HANDLE) theDevice;
	DWORD numRead = 0;
	COMMTIMEOUTS cto;
	int retry_count = 0;
	
	GetCommTimeouts(hCom, &cto);
	cto.ReadTotalTimeoutConstant = timeOut / 1000;
	SetCommTimeouts(hCom, &cto);
	
	while (retry_count < maxRetries && numRead == 0) {
		ReadFile(hCom, theBytes, maxBytes, &numRead, NULL);
		if (numRead > 0) {
			break;  // Success
		}
		retry_count++;
		if (retry_count < maxRetries) {
			Sleep(5);  // 5ms delay between retries
		}
	}
	
	return (unsigned int) numRead;
#endif
}

// Write theBytes to theDevice.
void WriteBytes(int theDevice, unsigned char *theBytes, unsigned int numBytes)
{
#ifndef WIN32
	unsigned int	i;

	if (!suppressWrite)
		write(theDevice, theBytes, numBytes);
#else
	DWORD				ret;
	unsigned int	i;

	if (!suppressWrite)
		WriteFile((HANDLE) theDevice, theBytes, numBytes, &ret, NULL );
#endif

	if (comm_debug)
	{
		for (i=0; i<numBytes; i++)
		{
			fprintf(comm_debug, " O-0x%02x", theBytes[i]);
			comm_debug_count++;

			if (comm_debug_count >= 8)
			{
				comm_debug_count = 0;
				fprintf(comm_debug, "\n");
			}
		}
/*
		if (comm_debug_count)
		{
			comm_debug_count = 0;
			fprintf(comm_debug, "\n");
		}
*/
	}
}

// Flush any bytes that may be waiting at theDevice
void FlushBytes(int theDevice)
{
	if (!suppressWrite)
	{
#ifndef WIN32
		tcflush(theDevice, TCIOFLUSH);								// flush the input stream
#else
		PurgeComm ((HANDLE) theDevice, PURGE_RXCLEAR | PURGE_RXABORT);
		PurgeComm ((HANDLE) theDevice, PURGE_TXCLEAR | PURGE_TXABORT);
#endif
	}
}

// set up data transmission configuration of theDevice
// NOTE: if any of the passed parameters is invalid, it will be set
// to an arbitrary valid value
// baudRate is: 50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
// dataBits is 7 or 8
// stopBits is 1 or 2
// parity is 0=none,
//           1=odd,
//           2=even
// if there is a problem, return false

#ifdef WIN32
//#define	B50		CBR_50
//#define	B75		CBR_75
#define	B110		CBR_110
#define	B300		CBR_300
#define	B600		CBR_600
#define	B1200		CBR_1200
//#define	B1800		CBR_1800
#define	B2400		CBR_2400
#define	B4800		CBR_4800
#define	B9600		CBR_9600
#define	B14400	CBR_14400
#define	B19200	CBR_19200
#define	B38400	CBR_38400
#define	B56000	CBR_56000
#define	B57600	CBR_57600
#define	B115200	CBR_115200
//#define	B128000	CBR_128000
//#define	B256000	CBR_256000
#endif

bool ConfigureDevice(int theDevice, unsigned int baudRate, unsigned char dataBits, unsigned char stopBits, unsigned char parity, bool cooked)
{
#ifndef WIN32
	struct termios	terminalParams;
	speed_t			theSpeed;
#else
	DCB				dcb;
	unsigned int	theSpeed;
#endif

#ifndef WIN32
	tcflush(theDevice, TCIOFLUSH);					// flush any unwritten, unread data
#else
	GetCommState((HANDLE) theDevice, &dcb);
#endif

#ifndef WIN32
	if (tcgetattr(theDevice, &terminalParams) != -1)	// read the old value
	{
#endif
		switch (baudRate)
		{
/*
			case 5:
				theSpeed = B50;	break;
			case 75:
				theSpeed = B75;	break;
			case 110:
				theSpeed = B110;	break;
			case 134:
				theSpeed = B134;	break;
			case 150:
				theSpeed = B150;	break;
			case 200:
				theSpeed = B200;	break;
*/
			case 300:
				theSpeed = B300;	break;
			case 600:
				theSpeed = B600;	break;
			case 1200:
				theSpeed = B1200;	break;
//			case 1800:
//				theSpeed = B1800;	break;
			case 2400:
				theSpeed = B2400;	break;
			case 4800:
				theSpeed = B4800;	break;
			case 9600:
				theSpeed = B9600;	break;
			case 19200:
				theSpeed = B19200;	break;
			case 38400:
				theSpeed = B38400;	break;
			case 57600:
				theSpeed = B57600;	break;
			case 115200:
				theSpeed = B115200;	break;
//			case 128000:
//				theSpeed = B128000;	break;
//			case 256000:
//				theSpeed = B256000;	break;
			default:
				theSpeed = B19200;	break;
		}

#ifndef WIN32

		cfsetospeed(&terminalParams, theSpeed);
		cfsetispeed(&terminalParams, theSpeed);

		terminalParams.c_cflag &= ~CSIZE;		// mask off the data bits

		switch (dataBits)
		{
			case 7:
				terminalParams.c_cflag |= CS7;
				terminalParams.c_iflag |= ISTRIP;
				break;
			case 8:
			default:
				terminalParams.c_cflag |= CS8;
				terminalParams.c_iflag &= ~ISTRIP;
				break;
		}

		switch (stopBits)
		{
			case 2:
				terminalParams.c_cflag |= CSTOPB;
				break;
			case 1:
			default:
				terminalParams.c_cflag &= ~CSTOPB;
				break;
		}

		switch (parity)
		{
			case 0:
				terminalParams.c_cflag &= ~(PARENB | PARODD);	// no parity
				terminalParams.c_iflag &= ~INPCK;
				break;
			case 1:
				terminalParams.c_cflag |= (PARENB | PARODD);		// odd parity
				terminalParams.c_iflag |= INPCK;
				break;
			case 2:
				terminalParams.c_cflag |= PARENB;					// even parity
				terminalParams.c_cflag &= ~PARODD;
				terminalParams.c_iflag |= INPCK;
				break;
			default:
				break;
		}

		if (cooked)							// use this when setting up the serial port to be used as a terminal
		{
			terminalParams.c_iflag = ICRNL;
			terminalParams.c_oflag = OPOST | ONLCR;
			terminalParams.c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHONL;
		}
		else
		{
			// *** Input flag settings. ***
			terminalParams.c_iflag &= ~(IGNCR | INLCR | ICRNL | IXOFF | IXON);

			// *** Output flag settings. ***
			terminalParams.c_oflag &= ~OPOST;

			// *** Control flag settings. ***
			terminalParams.c_cflag &= ~HUPCL;
			terminalParams.c_cflag |= (CLOCAL | CREAD);

			// *** Local flag settings. ***
			terminalParams.c_lflag &= ~(ICANON|ECHO|ECHONL|ISIG|IEXTEN|TOSTOP);

			terminalParams.c_cc[VINTR]  = _POSIX_VDISABLE;
			terminalParams.c_cc[VQUIT]  = _POSIX_VDISABLE;
			terminalParams.c_cc[VSUSP]  = _POSIX_VDISABLE;
			terminalParams.c_cc[VSTART] = _POSIX_VDISABLE;
			terminalParams.c_cc[VSTOP]  = _POSIX_VDISABLE;
		}

		terminalParams.c_cc[VMIN] = MIN_CHARS;			// wait until at least one character or until timeout
		terminalParams.c_cc[VTIME] = CHAR_TIMEOUT;	// wait this long, then fail

		if (tcsetattr(theDevice, TCSANOW, &terminalParams) != -1) 
			return(true);

#else

	switch (dataBits)
	{
		case 7: dataBits = 7; break;
		case 8:
		default: dataBits = 8;
	}

	switch (stopBits)
	{
		case 2: stopBits = TWOSTOPBITS; break;
		case 1: 
		default: stopBits = ONESTOPBIT; break;
	}

	switch (parity)
	{
		case 1: parity = ODDPARITY; break;
		case 2: parity = EVENPARITY; break;
		case 0: 
		default: parity = NOPARITY; break;
	}

	dcb.BaudRate = theSpeed;
	dcb.Parity = parity;
	dcb.StopBits = stopBits;
	dcb.ByteSize = dataBits;
#endif

#ifndef WIN32
	}

	return(false);
#else
	return SetCommState((HANDLE) theDevice, &dcb);
#endif
}

// return the configuration of theDevice
// baudRate is: 50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200,230400,460800
// dataBits is 7 or 8
// stopBits is 1 or 2
// parity is 0=none,
//           1=odd,
//           2=even
void GetDeviceConfiguration(int theDevice, unsigned int *baudRate, unsigned char *dataBits, unsigned char *stopBits, unsigned char *parity)
{
#ifndef WIN32
	struct termios	terminalParams;

	if (tcgetattr(theDevice, &terminalParams) != -1)	// read the old value
	{
		switch (cfgetospeed(&terminalParams))
		{
/*
			case B50:
				*baudRate = 50;	break;
			case B75:
				*baudRate = 75;	break;
			case B110:
				*baudRate = 110;	break;
			case B134:
				*baudRate = 134;	break;
			case B150:
				*baudRate = 150;	break;
			case B200:
				*baudRate = 200;	break;
*/
			case B300:
				*baudRate = 300;	break;
			case B600:
				*baudRate = 600;	break;
			case B1200:
				*baudRate = 1200;	break;
//			case B1800:
//				*baudRate = 1800;	break;
			case B2400:
				*baudRate = 2400;	break;
			case B4800:
				*baudRate = 4800;	break;
			case B9600:
				*baudRate = 9600;	break;
			case B19200:
				*baudRate = 19200;	break;
			case B38400:
				*baudRate = 38400;	break;
			case B57600:
				*baudRate = 57600;	break;
			case B115200:
				*baudRate = 115200;	break;
//			case B128000:
//				 *baudRate = B128000;	break;
//			case B256000:
//				*baudRate = B256000;	break;
			default:
				*baudRate = 0;	break;
		}

		switch (terminalParams.c_cflag & CSIZE)
		{
			case CS7:
				*dataBits = 7;	break;
			case CS8:
				*dataBits = 8;	break;
			default:
				*dataBits = 0;	break;
		}

		*stopBits = 1;

		if (terminalParams.c_cflag & CSTOPB)
			*stopBits = 2;

		*parity = 0;

		if (terminalParams.c_cflag & PARENB)
		{
			if (terminalParams.c_cflag & PARODD)
				*parity = 1;
			else
				*parity = 2;
		}
	}

#else		// WIN32
	DCB	dcb;

	GetCommState((HANDLE) theDevice, &dcb);
	*baudRate = dcb.BaudRate;
	*dataBits = dcb.ByteSize;

	switch (dcb.StopBits)
	{
		case ONESTOPBIT:
			*stopBits = 1; break;
		case TWOSTOPBITS:
			*stopBits = 2; break;
	}

	switch (dcb.Parity)
	{
		case EVENPARITY:
			*parity = 2; break;
		case ODDPARITY:
			*parity = 1; break;
		case NOPARITY:
			*parity = 0; break; 
	}
#endif
}

// if wantControl is true, configure theDevice to use CTS/RTS hardware
// flow control, if false, turn off flow control
// NOTE: when flow control is off, we must drive our RTS line to the
// modem active at all times
// if there is a problem, return false
bool ConfigureFlowControl(int theDevice, bool wantControl)
{
#ifndef WIN32
	struct termios	terminalParams;

	if (tcgetattr(theDevice, &terminalParams) != -1)	// read the old value
	{
		if (wantControl)
			terminalParams.c_cflag |= CRTSCTS;		// set flow control
		else
			terminalParams.c_cflag &= ~CRTSCTS;		// clear flow control

		terminalParams.c_cc[VMIN] = MIN_CHARS;		// read returns immediately if no characters
		terminalParams.c_cc[VTIME] = CHAR_TIMEOUT;

		if (tcsetattr(theDevice, TCSANOW, &terminalParams) != -1) 
			return(true);
	}

	return(false);

#else		// WIN32
	DCB dcb;

	GetCommState((HANDLE) theDevice, &dcb);

	if (wantControl)
	{
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
		dcb.fOutxCtsFlow = true;
	}
	else
	{
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fOutxCtsFlow = false;
	}

	return SetCommState((HANDLE) theDevice, &dcb);
#endif
}

// return the status of the two control lines we are interested in:
// CTS is true when the modem has raised CTS (letting us know it is ok to send)
// DCD is true when the carrier detect line is active
void GetDeviceStatus(int theDevice, bool *CTS, bool *DCD)
{
#ifndef WIN32
	int	status;

	*CTS = *DCD = false;

	if (ioctl(theDevice, TIOCMGET, &status) != -1)
	{
		if (status & TIOCM_CTS)
			*CTS = true;

		if(status & TIOCM_CAR)
			*DCD = true;
	}
#else
	DWORD modemStat;

	GetCommModemStatus((HANDLE) theDevice, &modemStat);
	*CTS = ((modemStat & MS_CTS_ON) > 0);
	*DCD = ((modemStat & MS_RLSD_ON) > 0);
#endif
}

// Set the state of the DTR handshake line
void SetDTR(int theDevice, bool DTR)
{
#ifndef WIN32
	int	control;

	control = TIOCM_DTR;

	if (DTR)
		ioctl(theDevice, TIOCMBIS, &control);
	else
		ioctl(theDevice, TIOCMBIC, &control);
#else
	DCB dcb;

	GetCommState((HANDLE) theDevice, &dcb);

	if (DTR)
		dcb.fDtrControl =DTR_CONTROL_ENABLE;
	else
		dcb.fDtrControl = DTR_CONTROL_DISABLE;

	SetCommState((HANDLE) theDevice, &dcb);
#endif
}

#ifndef WIN32
static struct termios oldTerminalParams;
#else
static DCB				oldDcb;
static COMMTIMEOUTS	oldCto;
#endif

// Open theName immediately for both read/write
// (do not block under any circumstances)
// return a device handle.
// NOTE: since the device can be opened BEFORE it is locked,
// this function MUST NOT modify the parameters of the device
// or in any way mess with it!!
// if there is a problem, set the error, and return false
bool OpenDevice(char *theName, int *theDevice)
{
#ifndef WIN32
        *theDevice = open(theName, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (*theDevice < 0) {
                return *theDevice;
        }
        
        // Fix K150 read issues: Make serial port blocking for reliable reads
        int flags = fcntl(*theDevice, F_GETFL);
        if (flags != -1) {
            fcntl(*theDevice, F_SETFL, flags & ~O_NONBLOCK);
        }

        // attempt to read configuration, to verify this is a serial device
        // and to save settings
        if (tcgetattr(*theDevice, &oldTerminalParams) != -1)
            return(true);

	return(false);
#else
	HANDLE hCom;
	DCB dcb;
	COMMTIMEOUTS cto;

	hCom = CreateFile(theName, GENERIC_READ | GENERIC_WRITE, 0, /* not shared */ NULL, OPEN_EXISTING, 0, NULL);

	if (hCom == INVALID_HANDLE_VALUE)
		return false;

	GetCommState(hCom, &oldDcb);
	GetCommState(hCom, &dcb);

	memset(&dcb, 0, sizeof(dcb));

	dcb.DCBlength  			= sizeof(dcb);		 		// sizeof(DCB)
	dcb.BaudRate				= CBR_19200;				// current baud rate
	dcb.fBinary 				= TRUE;						//
	dcb.fParity 				= FALSE;						// disable parity checking
	dcb.fOutxCtsFlow  		= FALSE;						// no CTS output flow control
	dcb.fOutxDsrFlow  		= FALSE;						// no DSR output flow control
	dcb.fDtrControl			= DTR_CONTROL_DISABLE;	// DTR flow control type
	dcb.fDsrSensitivity  	= FALSE;						// DSR sensitivity
	dcb.fTXContinueOnXoff	= TRUE; 						// XOFF continues Tx
	dcb.fOutX					= FALSE;						// XON/XOFF out flow control
	dcb.fInX 					= FALSE;						// XON/XOFF in flow control
	dcb.fNull					= FALSE;						// enable null stripping
	dcb.fRtsControl			= RTS_CONTROL_DISABLE;	// RTS flow control
	dcb.fAbortOnError 		= FALSE;						// abort reads/writes on error
	dcb.ByteSize				= 8;							// number of bits/byte, 4-8
	dcb.Parity  				= NOPARITY;					// 0-4=no,odd,even,mark,space
	dcb.StopBits				= ONESTOPBIT;				// 0,1,2 = 1, 1.5, 2

	if (SetCommState(hCom, &dcb) == 0)
	{
		CloseHandle(hCom);
		return false;  	
	}

	GetCommTimeouts(hCom, &oldCto);
	GetCommTimeouts(hCom, &cto);

	cto.ReadIntervalTimeout = 0;
	cto.ReadTotalTimeoutMultiplier = 0;
	cto.ReadTotalTimeoutConstant = 1;
	cto.WriteTotalTimeoutMultiplier = 0;
	cto.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(hCom, &cto);

	*theDevice = (int) hCom;

	return true;	
#endif
}

// Close theDevice
void CloseDevice(int theDevice)
{
	// try to set the parameters back as they were, don't care if we fail
#ifndef WIN32
	tcsetattr(theDevice, TCSANOW, &oldTerminalParams);
	close(theDevice);
#else
	SetCommTimeouts((HANDLE) theDevice, &oldCto);
	SetCommState((HANDLE) theDevice, &oldDcb);
	CloseHandle((HANDLE) theDevice);
#endif
}

