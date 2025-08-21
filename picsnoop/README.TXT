picsnoop -- version 0.1b, 2 November 2002
(Modified for Windows December 2004 - JLP)

  this program is used to snoop the communications between MPLAB (running on
    a PC under Windows) and a PICSTART Plus.  The resulting information can
    be used to create device profiles (stored in picdev.c) to allow picp to
    support new devices.

  what you need:
     1) a PICSTART Plus
     2) a PC running the latest version of MPLAB (that supports the devices in question)
     3) a linux box with one serial port
     4) a serial cable tap (this is the part you need to make)

   The basic idea is that the serial port on the linux box eavesdrops on the communication
from MPLAB to the PICSTART Plus and dumps the device profile in a format that can be pasted
into picdev.c.

MAKING THE CABLE
----------------

To make the tap cable you need three DB-9 connectors: 2 female and 1 male.  Wire them as follows:

   female                                male
    ----    RxD                          ----  T
   |  2 |-------------------------------|  2 | O
 T |    |   TxD                         |    |
 O |  3 |---------------x---------------|  3 | P
   |    |   GND         |               |    | I
 P |  5 |---------------------x---------|  5 | C
 C |    |   RTS         |     |         |    | S
   |  7 |-------------------------------|  7 | T
   |    |   CTS         |     |         |    | A
   |  8 |-------------------------------|  8 | R
    ----                |     |          ----  T
                        |     |
                        |     |
   female               |     |
    ----                |     |
 S |  2 |---------------+     |
 N |    |                     |
 O |  3 |                     |
 O |    |                     | this connector snoops data sent by the PC to the PICSTART Plus
 P |  5 |---------------------+
   |    |
 T |  7 |
 X |    |
 D |  8 |
    ----

The other pins may be ignored.

Connect the snoop connector to the serial port on the Linux box.

BUILDING PICSNOOP
-----------------

Unpack the archive:
  tar -xzf picnoop.tgz

Build it:
  make

Install it (you'll probably need to be root):
  make install

picsnoop installs itself at /usr/local/bin.

RUNNING PICSNOOP
----------------

First launch MPLAB and be sure it can communicate with the PICSTART Plus (select "Enable Programmer"
from the PICSTART Plus pull-down menu).  It should open the "PICSTART Plus Device Programmer" dialog
box if all is well.

Next, invoke picsnoop:
  picsnoop /dev/ttyS0
  picsnoop com1  (Windows)

Substitute the correct device if different.  picsnoop will open the port with flow control disabled
(no hardware handshaking), 19.2Kbaud, 8 data bits, 1 start bit, 1 stop bit, and no parity.

MPLAB sends the device profile to the PICSTART Plus whenever you select a new device from the "Device"
pull-down in the "PICSTART Plus Device Programmer" dialog.  picsnoop captures the profile as it goes
past and dumps it out in the format that picdev.c expects, like this:

//-----------------------------------------------------------
const static unsigned char def_PICxxxxx[] =
{
        0x01, 0xff,             // size of program space
        0x0f, 0xff,             // width of address word
        0x0f, 0xff,             // width of data word
        0x00, 0x0f,             // ??
        0x00, 0x0f,             // ??
        0x00, 0x1f,             // width of configuration word
        0x00, 0x1f,             // configuration word mask
        0x00, 0x00,             // ??
        0x00, 0x00,             // ??
        0x0f, 0xff,             // ??
        0x0f, 0xff,             // ??
        0x00, 0x00,             // ??
        0x00, 0x01,             // ??
        0x02, 0x00,             // ??
        0x04,                   // ??
        0x0f, 0xff,             // address of configuration bits
        0x01,                   // size of configuration register
        0x00, 0x00,             // address of data space
        0x00, 0x00,             // size of data space
        0x01, 0xff,             // address of internal clock calibration value
        0x00, 0x01,             // size of clock calibration space
        0x0b, 0x08,             // ??
        0x0a, 0x16,             // ??
};

const static unsigned char defx_PICxxxxx[] =
{
        0x00, 0x1f, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x1f, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
};

const static PIC_DEFINITION PICxxxxx =
{
        "xxxxx",                // device name
        def_PICxxxxx,           // definition
        defx_PICxxxxx,          // extended definition
        0,                      // config word: code protect bit mask
        0,                      // config word: watchdog bit mask

        0,                      // Word alignment for writing to this device
        0,                      // Configuration memory start address
        0, 0,                   // ID Locations addr
        0                       // Data eeprom address
};

Since the device name is not sent picsnoop doesn't know what it is, so it substitutes "xxxxx".
To add this device to picdev.c, cut and paste this block then change the "xxxxx" to the proper
device name (like "12C508", in this case).  Finally, add the device to the deviceList array at
the end of picdev.c:
	&PICxxxxx,
Again, substitute the device name for "xxxxx".


