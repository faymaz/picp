// serial.c
// Handle opening and conditioning of the serial port
// These functions isolate all communications with the
// serial device

#include	<stdio.h>
#include	<ctype.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include	<dos.h>

#include	"serial.h"

#define	byte	unsigned char
#define	word	unsigned short

#define	BUFLEN	2048					/* receive buffer length */
#define	ERRCODES	(OVRERR | PARERR | FRMERR | BRKINT)

void	interrupt comintr1(void);
void	interrupt comintr2(void);
void	init_com(int device, int baud, int param);
int	getcom1(void);
int	getcom2(void);

/* Globals */

int	commerr1;						/* error flags */
int	commerr2;
char	combfr1[BUFLEN];				/* receive buffers */
char	combfr2[BUFLEN];
char	*getptr1;						/* buffer pointers for intrpt routines */
char	*putptr1;
char	*getptr2;
char	*putptr2;
void	interrupt(*oldvect1)();		/* place to save old interrupt vector */
void	interrupt(*oldvect2)();

// set up data transmission configuration of theDevice
// NOTE: if any of the passed parameters is invalid, it will be set
// to an arbitrary valid value
// baudRate is: 1200,2400,4800,9600,19200,38400,57600,115200
// dataBits is 7 or 8
// parity is 0=none,
//           1=odd,
//           2=even
// if there is a problem, return false

bool ConfigureDevice(int theDevice, unsigned int baudRate, unsigned char dataBits, unsigned char parity)
{
	int	baud, bits, param;

	switch (baudRate)
	{
		case 1200:
			baud = BAUD12;		/* 1200 baud */
			break;

		case 2400:
			baud = BAUD24;		/* 2400 baud */
			break;

		case 4800:
			baud = BAUD48;		/* 4800 baud */
			break;

		case 9600:
			baud = BAUD96;		/* 9600 baud */
			break;

		case 19200:
			baud = BAUD19;		/* 19200 */
			break;

		case 38400:
			baud = BAUD38;		/* 38400 */
			break;

		case 57600:
			baud = BAUD57;		/* 57600 */
			break;

		case 115200:
			baud = BAUD11;		/* 115200 */
			break;

		default:
			baud = 0;
			break;
	}

	switch (dataBits)
	{
		case 7:
			bits = WLEN7;
			break;

		case 8:
			bits = WLEN8;
			break;

		default:
			bits = 0;
			break;
	}

	switch (parity)
	{
		case 0:
			param = 0;
			break;

		case 1:
			param = PARODD;
			break;

		case 2:
			param = PAREVN;
			break;

		default:
			break;
	}

	if (!bits || !baud)
		return false;

	param = param | bits;
	init_com(theDevice, baud, param);
	return true;
}

bool compareStr(char *s1, char *s2)
{
	char	a, b;
	bool	match = true;

	while (*s1 && *s2)
	{
		a = tolower(*s1);
		b = tolower(*s2);
		s1++;
		s2++;

		if (a != b)			// character doesn't match
		{
			match = false;
			break;
		}
	}

	if (*s1 != *s2)		// must be same length
		match = false;

	return match;
}

// Open theName immediately for both read/write
// (do not block under any circumstances)
// return a device handle.
// NOTE: since the device can be opened BEFORE it is locked,
// this function MUST NOT modify the parameters of the device
// or in any way mess with it!!
// if there is a problem, set the error, and return false

bool OpenDevice(char *theName, int *theDevice)
{
	if (compareStr(theName, "com1"))
		*theDevice = 1;
	else if (compareStr(theName, "com2"))
		*theDevice = 2;
	else
	{
		*theDevice = 0;
		return false;
	}

	return true;
}

// Close theDevice

void CloseDevice(int theDevice)
{
	if (theDevice == 1)
		setvect(INUM1, oldvect1);
	else if (theDevice == 2)
		setvect(INUM2, oldvect2);
}

/*  INTERRUPT ROUTINES */

void interrupt comintr1(void)
{
	(char) *putptr1 = inportb(DPORT1);		/* get data into buffer */
	putptr1++;										/* bump buffer pointer */

	if (putptr1 == combfr1 + BUFLEN)
		putptr1 = combfr1;

	commerr1 = inportb(LSTAT1) & ERRCODES;	/* set any error bits */
	outportb(INTCTL, ENDINT);					/* make 8259 happy */
}

void interrupt comintr2(void)
{
	(char) *putptr2 = inportb(DPORT2);		/* get data into buffer */
	putptr2++;										/* bump buffer pointer */

	if (putptr2 == combfr2 + BUFLEN)
		putptr2 = combfr2;

	commerr2 = inportb(LSTAT2) & ERRCODES;	/* set any error bits */
	outportb(INTCTL, ENDINT);					/* make 8259 happy */
}

/*  Initialize comm ports */

void init_com(int device, int baud, int param)
{
	char	mask;
	int	lcreg, divll, divlh, mctrl, intena, sioena;

	getptr2 = combfr2;				/* initialize buffer pointers */
	putptr2 = combfr2;
	getptr1 = combfr1;
	putptr1 = combfr1;
	commerr2 = 0;						/* clear error flag */
	commerr1 = 0;

	if (device == 2)
	{
		oldvect2 = getvect(INUM2);		/* get old vector routine address */
		setvect(INUM2, &comintr2);		/* install interrupt vector */
		lcreg = LCREG2;
		divll = DIVLL2;
		divlh = DIVLH2;
		mctrl = MCTRL2;
		intena = INTENA2;
		sioena = SIOENA2;
		outportb(lcreg, param | DLAB);	/* set params & div latch access */
		outportb(divll, baud);
		outportb(divlh, 0);
		outportb(lcreg, param);				/* clear div latch access */
		outportb(mctrl, MDTR | MRTS | MOUT2);
												/* enable dtr, rts, and interrupts */
		outportb(intena, RCVENA);			/* enable receive interrupts */
		mask = inportb(INTMSK);				/* tell 8259 about it */
		outportb(INTMSK, mask & sioena);
	}
	else if (device == 1)
	{
		oldvect1 = getvect(INUM1);		/* get old vector routine address */
		setvect(INUM1, &comintr1);		/* install interrupt vector */
		lcreg = LCREG1;
		divll = DIVLL1;
		divlh = DIVLH1;
		mctrl = MCTRL1;
		intena = INTENA1;
		sioena = SIOENA1;
		outportb(lcreg, param | DLAB);	/* set params & div latch access */
		outportb(divll, baud);
		outportb(divlh, 0);
		outportb(lcreg, param);				/* clear div latch access */
		outportb(mctrl, MDTR | MRTS | MOUT2);
												/* enable dtr, rts, and interrupts */
		outportb(intena, RCVENA);			/* enable receive interrupts */
		mask = inportb(INTMSK);				/* tell 8259 about it */
		outportb(INTMSK, mask & sioena);
	}
}

/*	Get next character from receive buffer.
	Returns 0 if no character available else returns
	character (0x100 if received character = 0).
	If comm error occurs, bit 9 will be set.
*/

int getcom1(void)
{
	int	c;

	if (putptr1 != getptr1)
	{
		c = *getptr1++;

		if (getptr1 == combfr1 + BUFLEN)
			getptr1 = combfr1;

		if (commerr1)
		{
			c = c | 0x200;
			commerr1 = 0;
		}

		if (c == 0)
			return(0x100);
		else
			return(c);
	}
	else
		return(0);
}

int getcom2(void)
{
	int	c;

	if (putptr2 != getptr2)
	{
		c = *getptr2++;

		if (getptr2 == combfr2 + BUFLEN)
			getptr2 = combfr2;

		if (commerr2)
		{
			c = c | 0x200;
			commerr2 = 0;
		}

		if (c == 0)
			return(0x100);
		else
			return(c);
	}
	else
		return(0);
}

int read(int device, char *bfr, int max)
{
	int	num, chr;
	char	*gptr, *pptr;

	num = 0;

	if (device == 1)
	{
		pptr = putptr1;
		gptr = getptr1;
	}
	else if (device == 2)
	{
		pptr = putptr2;
		gptr = getptr2;
	}
	else
		return 0;

	while ((pptr != gptr) && (num < max))
	{
		if (device == 1)
			chr = getcom1();
		else if (device == 2)
			chr = getcom2();

		if (chr)
		{
			chr &= 0xff;
			num++;
			*bfr = (unsigned char) chr;
			bfr++;
		}
	}

	return num;
}

// end

