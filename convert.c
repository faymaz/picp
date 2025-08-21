
//
// Convert picdev.c to picdevrc file
//

#include	<stdio.h>
#include	<string.h>
#include	<ctype.h>

#define	MAXCHAR	256

typedef struct
{
	char						*name;		// name of the device
	unsigned char			*def;			// definition (initialization) (always 44 bytes long)
	unsigned char			*defx;		// extended definition (initialization) (always 32 bytes long)
	unsigned short int	cpbits;		// set of code protection bits in configuration word (0 = read protected)
	unsigned short int	wdbit;		// watchdog enable bit in configuration word (1 = enabled)
	unsigned short int	wordalign;	// Word alignment for writing to this device.
	unsigned int			cfgmem;		// Configuration Memory Start addr
	unsigned int			idaddr;		// Address of ID Locations area
	unsigned short int	idsize;		// size of ID Locations area
	unsigned int			eeaddr;		// Data EEPROM address
	unsigned char			fixedCfgBitsSize;	// number of words (1-8, 0 = no fixed bits)
	unsigned short int	fixedCfgBits[8];	// up to 8 words of config bits
	unsigned short int	pgm_support;	// bit map of supporting programmers
} PIC_DEFINITION;

char	picname[32];
unsigned char	picdef[64];
unsigned char	picdefx[32];
char	programmers[64];

PIC_DEFINITION	picdev = {
	(char *) &picname,
	(unsigned char *) &picdef,
	(unsigned char *) &picdefx,
};

FILE *fpin, *fpout;
char	buffer[MAXCHAR];

// Skip over type info in declaration.
// Return NULL if error.
// Will return a pointer to "PIC_DEFINITION",
// "def_PIC", or "defx_PIC" depending on type.

char *skipTypeInfo(char *str)
{
	char	*cptr;

	cptr = strstr(str, "PIC");		// find PIC info

	if (cptr)
	{
		if (*(cptr - 1) != ' ')		// is not "PIC_DEFINITION"
		{
			while (*cptr != ' ')		// back up to def
				--cptr;

			cptr++;		// now point to def or defx
		}
	}

	return cptr;
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

// Extract PIC type from line and store in picname.
// Return NULL if type not found (end of entries).
// On entry, cptr -> "PIC_DEFINITION"

char *getPicType(char *str)
{
	char	*end;

	str = skipText(str);
	str = skipWhitespace(str);

	if (!*str || *str == '*')		// end of entries
		return NULL;

	str += 3;					// skip over "PIC"
	end = skipText(str);

	if (end)
		*end = '\0';

	strcpy(picname, str);
	return str;
}

//
// Convert string into an unsigned int, deciphering the base.
//  	0bnnnnnnnnnnnn = binary (or 0Bnnnnnnnnnnnn)
//		0xnnnn = hex (or 0Xnnnn)
//		anything else is decimal
// Return updated string pointer.

char *atoi_base(char *str, unsigned int *result)
{
	int	base, digit;
		
	base = 10;							// assume decimal
	*result = 0;						// start at zero
	str = skipWhitespace(str);

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

		while (*str)
		{
			digit = toupper(*str++);	// force all upper case for hex digits
			digit = (digit >= 'A') ? digit - 'A' + 0x0a : digit - '0';	// convert to 0-9, A-F (or try to)

			if (digit >= 0 && digit < base)			// make sure digit can be represented in this base
				*result = *result * base + digit;	// shift up one order of magnitude, add in this digit
			else
				break;				// bad character, done
		}
	}

	return str;
}

// Extract list of supported programmers

void addProgrammers(char *str)
{
	int	offset = 0;

	while (str)
	{
		str = skipWhitespace(str);
		str = strstr(str, "P_");		// get programmer name

		if (str)
		{
			str += 2;

			if (offset)
				programmers[offset++] = ' ';

			while (isalpha(*str))
			{
				programmers[offset++] = *str++;
				programmers[offset] = '\0';
			}
		}
	}
}

// Last part of PIC definiton.
// Read into picdev and write the info to the output file.

void readPIC(void)
{
	int				i;
	unsigned int	val;
	char				*str;

	for (i=0; i<5; i++)		// skip next 4 lines
		fgets(buffer, MAXCHAR - 1, fpin);

	str = fgets(buffer, MAXCHAR - 1, fpin);	// get code protect bits
	atoi_base(str, &val);
	picdev.cpbits = (unsigned short int) val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get watchdog bit mask
	atoi_base(str, &val);
	picdev.wdbit = (unsigned short int) val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get word alignment
	atoi_base(str, &val);
	picdev.wordalign = (unsigned short int) val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get config mem address
	atoi_base(str, &val);
	picdev.cfgmem = val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get ID address
	atoi_base(str, &val);
	picdev.idaddr = val;
	str++;							// get ID size
	atoi_base(str, &val);
	picdev.idsize = (unsigned short int) val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get EEPROM address
	atoi_base(str, &val);
	picdev.eeaddr = val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get number of cfg words
	atoi_base(str, &val);
	picdev.fixedCfgBitsSize = (unsigned char) val;
	str = fgets(buffer, MAXCHAR - 1, fpin);	// get fixed bit masks
	str = skipWhitespace(str);

	for (i=0; i<8; i++)
	{
		str++;
		str = atoi_base(str, &val);
		picdev.fixedCfgBits[i] = (unsigned short int) val;
	}

	str = fgets(buffer, MAXCHAR - 1, fpin);	// get supported programmers
	addProgrammers(str);

	fprintf(fpout, "[%s]\t; pic definition\n", picname);

	fprintf(fpout, "\t%x\t; config word: code protect bit mask\n", picdev.cpbits);
	fprintf(fpout, "\t%x\t; config word: watchdog bit mask\n", picdev.wdbit);
	fprintf(fpout, "\t%x\t; Word alignment for writing to this device\n", picdev.wordalign);
	fprintf(fpout, "\t%x\t; Configuration memory start address\n", picdev.cfgmem);
	fprintf(fpout, "\t%x %x\t; ID Locations addr and size\n", picdev.idaddr, picdev.idsize);
	fprintf(fpout, "\t%x\t; Data eeprom address\n", picdev.eeaddr);
	fprintf(fpout, "\t%x\t; number of words in cfg bits with factory set bits\n\t", picdev.fixedCfgBitsSize);

	for (i=0; i<7; i++)
		fprintf(fpout, "%x ", picdev.fixedCfgBits[i]);

	fprintf(fpout, "%x\t; fixed bits mask\n", picdev.fixedCfgBits[7]);
	fprintf(fpout, "\t%s\t; bit map of supporting programmers\n", programmers);

	fprintf(fpout, "\n[%s:def]\n", picname);

	fprintf(fpout, "\t%02x %02x\t; size of program space\n",
		picdef[0], picdef[1]);
	fprintf(fpout, "\t%02x %02x\t; width of address word\n",
		picdef[2], picdef[3]);
	fprintf(fpout, "\t%02x %02x\t; width of data word\n",
		picdef[4], picdef[5]);
	fprintf(fpout, "\t%02x %02x\t; width of ID\n",
		picdef[6], picdef[7]);
	fprintf(fpout, "\t%02x %02x\t; ID mask\n",
		picdef[8], picdef[9]);
	fprintf(fpout, "\t%02x %02x\t; width of configuration word\n",
		picdef[10], picdef[11]);
	fprintf(fpout, "\t%02x %02x\t; configuration word mask\n",
		picdef[12], picdef[13]);
	fprintf(fpout, "\t%02x %02x\t; EEPROM data width\n",
		picdef[14], picdef[15]);
	fprintf(fpout, "\t%02x %02x\t; EEPROM data mask\n",
		picdef[16], picdef[17]);
	fprintf(fpout, "\t%02x %02x\t; Calibration width\n",
		picdef[18], picdef[19]);
	fprintf(fpout, "\t%02x %02x\t; Calibration mask\n",
		picdef[20], picdef[21]);
	fprintf(fpout, "\t%02x %02x\t; ??\n",
		picdef[22], picdef[23]);
	fprintf(fpout, "\t%02x %02x\t; ??\n",
		picdef[24], picdef[25]);
	fprintf(fpout, "\t%02x %02x\t; address of ID locations\n",
		picdef[26], picdef[27]);
	fprintf(fpout, "\t%02x\t; size of ID locations\n",
		picdef[28]);
	fprintf(fpout, "\t%02x %02x\t; address of configuration bits\n",
		picdef[29], picdef[30]);
	fprintf(fpout, "\t%02x\t; size of configuration register\n",
		picdef[31]);
	fprintf(fpout, "\t%02x %02x\t; address of data space\n",
		picdef[32], picdef[33]);
	fprintf(fpout, "\t%02x %02x\t; size of data space\n",
		picdef[34], picdef[35]);
	fprintf(fpout, "\t%02x %02x\t; address of internal clock calibration value\n",
		picdef[36], picdef[37]);
	fprintf(fpout, "\t%02x %02x\t; size of clock calibration space\n",
		picdef[38], picdef[39]);
	fprintf(fpout, "\t%02x\t; additional programming pulses for C devices\n",
		picdef[40]);
	fprintf(fpout, "\t%02x\t; main programming pulses for C devices\n",
		picdef[41]);
	fprintf(fpout, "\t%02x %02x\t; ?? ZIF configuration ??\n",
		picdef[42], picdef[43]);

	fprintf(fpout, "\n[%s:defx]\n", picname);

	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[0], picdefx[1], picdefx[2], picdefx[3]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[4], picdefx[5], picdefx[6], picdefx[7]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[8], picdefx[9], picdefx[10], picdefx[11]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[12], picdefx[13], picdefx[14], picdefx[15]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[16], picdefx[17], picdefx[18], picdefx[19]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[20], picdefx[21], picdefx[22], picdefx[23]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n",
		picdefx[24], picdefx[25], picdefx[26], picdefx[27]);
	fprintf(fpout, "\t%02x %02x %02x %02x\n\n",
		picdefx[28], picdefx[29], picdefx[30], picdefx[31]);
}

// Read extended info into picdev

void readDefx(void)
{
	int				i = 0;
	unsigned int	val;
	char				*str;

	fgets(buffer, MAXCHAR - 1, fpin);	// skip next line

	while (buffer[0] != '}')
	{
		str = fgets(buffer, MAXCHAR - 1, fpin);	// read next line

		if (!str)
			break;

		while (str && *str)
		{
			str = atoi_base(str, &val);

			if (str)
			{
				picdev.defx[i++] = (unsigned char) val;
				str++;
				str = skipWhitespace(str);

				if (*str == '/')
					break;
			}
		}
	}
}

// Read definition info into picdev

void readDef(void)
{
	int				i = 0;
	unsigned int	val;
	char				*str;

	fgets(buffer, MAXCHAR - 1, fpin);	// skip next line

	while (buffer[0] != '}')
	{
		str = fgets(buffer, MAXCHAR - 1, fpin);	// read next line

		if (!str)
			break;

		while (str && *str)
		{
			str = atoi_base(str, &val);

			if (str)
			{
				picdev.def[i++] = (unsigned char) val;
				str++;
				str = skipWhitespace(str);

				if (*str == '/')
					break;
			}
		}
	}
}

int main(void)
{
	char	*cptr;

	fpin = fopen("picdev.c", "r");

	if (!fpin)
	{
		fprintf(stderr, "\npicdev.c file not found.\n\n");
		return 1;
	}

	fpout = fopen("picdevrc", "w");

	if (!fpout)
	{
		fprintf(stderr, "\nCan't open picdevrc file.\n\n");
		return 1;
	}

	fprintf(fpout, ";\n");
	fprintf(fpout, "; Resource configuration file for picp\n");
	fprintf(fpout, ";\n\n");

	while (!feof(fpin))
	{
		fgets(buffer, MAXCHAR - 1, fpin);

		if (!strncmp(buffer, "const", 5))
		{
			cptr = skipTypeInfo(buffer);

			if (!strncmp(cptr, "PIC", 3))		// is PIC_DEFINITION
			{
				if (getPicType(cptr))
					readPIC();
				else				// end of entries
				{
					fprintf(fpout, "\n");
					break;
				}
			}
			else if (!strncmp(cptr, "defx", 4))	// is defx entry
				readDefx();
			else if (!strncmp(cptr, "def_", 4))	// is def entry
				readDef();
		}
	}

	fclose(fpin);
	fflush(fpout);
	fclose(fpout);

	return 0;
}

// end
