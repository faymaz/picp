// serial.c
// Handle opening and conditioning of the serial port
// These functions isolate all communications with the
// serial device

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>

#ifdef WIN32
#include	<windows.h>
#define	false	FALSE
#define	true	TRUE
#else
#include <termio.h>
#endif

#define	byte		unsigned char
#define	word		unsigned short int

#include	"serial.h"

// See if there is unread data waiting on theDevice.
// This is used to poll theDevice without reading any characters
// from it.
// As soon as there is data, or timeOut expires, return.
// NOTE: timeOut is measured in microseconds
// if timeOut is 0, this will return the status immediately
bool ByteWaiting(int theDevice, int timeOut)
{
#ifndef WIN32
	fd_set		readSet;
	struct timeval	timeVal;

	FD_ZERO(&readSet);					// clear the set
	FD_SET(theDevice,&readSet);		// add this descriptor to the set
	timeVal.tv_sec=timeOut/1000000;	// set up the timeout waiting for one to come ready
	timeVal.tv_usec=timeOut%1000000;

	if (select(FD_SETSIZE, &readSet, NULL, NULL, &timeVal) == 1)	// if our descriptor is ready, then report it
		return(true);

	return(false);
#else
	COMSTAT	comState;
	DWORD		errors;

	ClearCommError ((HANDLE) theDevice, &errors, &comState);
	return comState.cbInQue > 0;
#endif
}

// Attempt to read at least one byte from theDevice before timeOut.
// once any byte is seen, attempt to get any more which are pending
// up to maxBytes
// If timeOut occurs, return 0
int ReadBytes(int theDevice, byte *theBytes, int maxBytes, int timeOut)
{
#ifndef WIN32
	int	numRead = 0;
#else
	HANDLE			hCom = (HANDLE) theDevice;
	DWORD				numRead;
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
		if ((numRead = read(theDevice, theBytes, maxBytes)) > 0)		// get waiting bytes
			return(numRead);
		else
			numRead = 0;
	}
#endif

	return(numRead);
}

// Write theBytes to theDevice.
void WriteBytes(int theDevice, byte *theBytes, int numBytes)
{
#ifndef WIN32
	write(theDevice, theBytes, numBytes);
#else
	DWORD	ret;

	WriteFile((HANDLE) theDevice, theBytes, numBytes, &ret, NULL);
#endif
}

// Flush any bytes that may be waiting at theDevice
void FlushBytes(int theDevice)
{
#ifndef WIN32
	ioctl(theDevice,TCFLSH,0);		// flush the input stream
#else
	PurgeComm ((HANDLE) theDevice, PURGE_RXCLEAR | PURGE_RXABORT);
	PurgeComm ((HANDLE) theDevice, PURGE_TXCLEAR | PURGE_TXABORT);
#endif
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

bool ConfigureDevice(int theDevice, int baudRate, byte dataBits, byte stopBits, byte parity, bool cooked)
{
#ifndef WIN32
	struct termios	terminalParams;
	speed_t			theSpeed;
#else
	DCB	dcb;
	int	theSpeed;
#endif

#ifndef WIN32
	if (ioctl(theDevice, TCGETS, &terminalParams) != -1)	// read the old value
	{
#endif
		switch (baudRate)
		{
/*
			case 50:
				theSpeed = B50;
				break;
			case 75:
				theSpeed = B75;
				break;
			case 110:
				theSpeed = B110;
				break;
			case 134:
				theSpeed = B134;
				break;
			case 150:
				theSpeed = B150;
				break;
			case 200:
				theSpeed = B200;
				break;
*/
			case 300:
				theSpeed = B300;
				break;
			case 600:
				theSpeed = B600;
				break;
			case 1200:
				theSpeed = B1200;
				break;
//			case 1800:
//				theSpeed = B1800;
//				break;
			case 2400:
				theSpeed = B2400;
				break;
			case 4800:
				theSpeed = B4800;
				break;
			case 9600:
				theSpeed = B9600;
				break;
			case 19200:
				theSpeed = B19200;
				break;
			case 38400:
				theSpeed = B38400;
				break;
			case 57600:
				theSpeed = B57600;
				break;
			case 115200:
				theSpeed = B115200;
				break;
//			case 230400:
//				theSpeed = B230400;
//				break;
//			case 460800:
//				theSpeed = B460800;
//				break;
			default:
				theSpeed = B9600;
				break;
		}

#ifndef WIN32

		cfsetospeed(&terminalParams, theSpeed);
		cfsetispeed(&terminalParams, theSpeed);

		terminalParams.c_cflag &= ~CSIZE;		// mask off the data bits

		switch (dataBits)
		{
			case 7:
				terminalParams.c_cflag |= CS7;
				break;
			case 8:
			default:
				terminalParams.c_cflag |= CS8;
				break;
		}

		terminalParams.c_cflag &= ~CSTOPB;	// mask off the stop bits

		switch (stopBits)
		{
			case 1:
				break;
			case 2:
				terminalParams.c_cflag |= CSTOPB;
				break;
			default:
				break;
		}

		terminalParams.c_cflag &= ~(PARENB | PARODD);	// mask off the parity bits

		switch (parity)
		{
			case 0:
				break;
			case 1:
				terminalParams.c_cflag |= (PARENB | PARODD);	// odd parity
				break;
			case 2:
				terminalParams.c_cflag |= PARENB;		// even parity
				break;
			default:
				break;
		}

		terminalParams.c_cflag |= CREAD;		// allow reading

		if (cooked)							// use this when setting up the serial port to be used as a terminal
		{
			terminalParams.c_iflag = ICRNL;
			terminalParams.c_oflag = OPOST | ONLCR;
			terminalParams.c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHONL;
		}
		else
		{
			terminalParams.c_iflag = 0;
			terminalParams.c_oflag = 0;
			terminalParams.c_lflag = 0;
		}

		terminalParams.c_cc[VMIN] = 0;		// read returns immediately if no characters
		terminalParams.c_cc[VTIME] = 0;

		if (ioctl(theDevice, TCSETS, &terminalParams) != -1)
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
void GetDeviceConfiguration(int theDevice, int *baudRate, byte *dataBits, byte *stopBits, byte *parity)
{
#ifndef WIN32
	struct termios	terminalParams;

	if (ioctl(theDevice, TCGETS, &terminalParams) != -1)	// read the old value
	{
		switch (cfgetospeed(&terminalParams))
		{
			case B50:
				*baudRate = 50;
				break;
			case B75:
				*baudRate = 75;
				break;
			case B110:
				*baudRate = 110;
				break;
			case B134:
				*baudRate = 134;
				break;
			case B150:
				*baudRate = 150;
				break;
			case B200:
				*baudRate = 200;
				break;
			case B300:
				*baudRate = 300;
				break;
			case B600:
				*baudRate = 600;
				break;
			case B1200:
				*baudRate = 1200;
				break;
			case B1800:
				*baudRate = 1800;
				break;
			case B2400:
				*baudRate = 2400;
				break;
			case B4800:
				*baudRate = 4800;
				break;
			case B9600:
				*baudRate = 9600;
				break;
			case B19200:
				*baudRate = 19200;
				break;
			case B38400:
				*baudRate = 38400;
				break;
			case B57600:
				*baudRate = 57600;
				break;
			case B115200:
				*baudRate = 115200;
				break;
			case B230400:
				*baudRate = 230400;
				break;
			case B460800:
				*baudRate = 460800;
				break;
			default:
				*baudRate = 0;
				break;
		}

		switch (terminalParams.c_cflag & CSIZE)
		{
			case CS7:
				*dataBits = 7;
				break;
			case CS8:
				*dataBits = 8;
				break;
			default:
				*dataBits = 0;
				break;
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

	if (ioctl(theDevice, TCGETS, &terminalParams) != -1)	// read the old value
	{
		if (wantControl)
			terminalParams.c_cflag |= CRTSCTS;		// set flow control
		else
			terminalParams.c_cflag &= ~CRTSCTS;		// clear flow control

		terminalParams.c_cc[VMIN] = 0;				// read returns immediately if no characters
		terminalParams.c_cc[VTIME] = 0;

		if (ioctl(theDevice, TCSETS, &terminalParams) != -1)
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

		if (status & TIOCM_CAR)
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

#ifdef WIN32
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
	struct termios	terminalParams;

	if ((*theDevice = open(theName, O_NDELAY | O_RDWR | O_NOCTTY)) != -1)	// NOTE: the NOCTTY will prevent us from grabbing this terminal as our controlling terminal (when run from init, we have no controlling terminal, an we do not want this device to become one!)
	{
		if (ioctl(*theDevice, TCGETS, &terminalParams) != -1)		// attempt to read configuration, just to verify this is a serial device
			return(true);

		close(*theDevice);
	}

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
#ifndef WIN32
	close(theDevice);
#else
	SetCommTimeouts((HANDLE) theDevice, &oldCto);
	SetCommState((HANDLE) theDevice, &oldDcb);
	CloseHandle((HANDLE) theDevice);
#endif
}
