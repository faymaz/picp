
#ifndef __SERIAL_H_
#define __SERIAL_H_

#define	bool	int

#define	KEYINT		int86(0x16, &regs, &regs)
#define	KBHEAD		0x41a
#define	KBTAIL		0x41c


#ifndef	FALSE
#define	FALSE	0
#endif
#ifndef	TRUE
#define	TRUE	1
#endif

#define	false	FALSE
#define	true	TRUE

#define	INUM1		12			// interrupt vector for com1
#define	INUM2		11			// interrupt vector for com2

#define	DPORT1	0x3f8		// receive/transmit data port
#define	DIVLL1	0x3f8		// divisor latch lsb
#define	DIVLH1	0x3f9		// divisor latch msb
#define	INTENA1	0x3f9		// interrupt enable register
#define	INTID1	0x3fa		// interrupt id register
#define	LCREG1	0x3fb		// line control register
#define	MCTRL1	0x3fc		// modem control register
#define	LSTAT1	0x3fd		// line status register
#define	MSTAT1	0x3fe		// modem status register

#define	DPORT2	0x2f8		// receive/transmit data port
#define	DIVLL2	0x2f8		// divisor latch lsb
#define	DIVLH2	0x2f9		// divisor latch msb
#define	INTENA2	0x2f9		// interrupt enable register
#define	INTID2	0x2fa		// interrupt id register
#define	LCREG2	0x2fb		// line control register
#define	MCTRL2	0x2fc		// modem control register
#define	LSTAT2	0x2fd		// line status register
#define	MSTAT2	0x2fe		// modem status register

#define	INTCTL	0x20		// 8259 interrupt controller address
#define	INTMSK	0x21		// interrupt enable mask register

//  Bit definitions

#define	SIOENA1	0xef		// mask enable
#define	SIOENA2	0xf7
#define	SIODIS1	0x10		// mask disable
#define	SIODIS2	0x08
#define	ENDINT	0x20		// end interrupt code

// LCREG - Line Control Register

#define	DLAB		0x80		// divisor latch access bit of lcreg
#define	WLEN5		0			// word length = 5 bits
#define	WLEN6		1			// 6 bits
#define	WLEN7		2			// 7 bits
#define	WLEN8		3			// 8 bits
#define	PARODD	8			// odd parity
#define	PAREVN	0x18		// even parity

// DIVLL - Divisor Latch (divlh = 0)

#define	BAUD12	0x60		// 1200 baud
#define	BAUD24	0x30		// 2400 baud
#define	BAUD48	0x18		// 4800 baud
#define	BAUD96	0x0c		// 9600 baud
#define	BAUD19	0x06		// 19200
#define	BAUD38	0x03		// 38400
#define	BAUD57	0x02		// 57600
#define	BAUD11	0x01		// 115200

// LSTAT - Line Status Register

#define	OVFERR	1			// buffer overflow error
#define	OVRERR	2			// overrun error
#define	PARERR	4			// parity error
#define	FRMERR	8			// framing error
#define	BRKINT	0x10		// break interrupt

// INTENA - Interrupt Enable Register

#define	RCVENA	1			// receive data
#define	XMTENA	2			// transmit
#define	LSTENA	4			// line status
#define	MSTENA	8			// modem status

// MCTRL - Modem Control Register

#define	MDTR		1			// data terminal ready
#define	MRTS		2			// request to send
#define	MOUT1		4			// aux output 1
#define	MOUT2		8			// aux output 2
#define	LPBACK	0x10		// hardware loopback

// MSTAT - Modem Status Register

#define	MDCTS		1			// delta clear to send
#define	MDDSR		2			// delta data set ready
#define	MDRING	4			// delta ring indicator
#define	MDLSD		8			// delta line signal detect
#define	MCTS		0x10		// clear to send
#define	MDSR		0x20		// data set ready
#define	MRING		0x40		// ring indicator
#define	MLSD		0x80		// line signal detect

// Prototypes

int	read(int device, char *bfr, int max);

bool	ConfigureDevice(int theDevice, unsigned int baudRate, unsigned char dataBits, unsigned char parity);
bool	OpenDevice(char *theName, int *theDevice);
void	CloseDevice(int theDevice);

#endif // defined __SERIAL_H_

