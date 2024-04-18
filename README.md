# picp
sudo apt-get install sdcc
sudo apt-get install picp
wget http://home.pacbell.net/theposts/picmicro/picp-0.6.8.tar.gz
wget https://launchpadlibrarian.net/26515430/picp_0.6.8.orig.tar.gz
tar -xzf picp-0.6.8.tar.gz
tar -xzf picp_0.6.8.orig.tar.gz
cd picp-0.6.8
make
sudo make install



$ picp -h   

picp: version 0.6.8
 (c) 2000-2004 Cosmodog, Ltd. (http://www.cosmodog.com)
 (c) 2004-2006 Jeff Post (http://home.pacbell.net/theposts/picmicro)
 GNU General Public License

Usage: picp [-c] [-d] [-v] ttyname [-v] devtype [-i] [-h] [-q] [-v] [-s [size]] [-b|-r|-w|-e][pcidof]
 where:
  ttyname is the serial (or USB) device the programmer is attached to
     (e.g. /dev/ttyS0 or com1)
  devtype is the pic device to be used (12C508, 16C505, etc.)
  -b blank checks the requested region or regions
  -c enable comm line debug output to picpcomm.log (must be before ttyname)
  -d (if only parameter) show device list
  -d devtype - show device information
  -e erases the requested region (flash parts only)
  -f ignores verify errors while writing
  -h show this help
  -i use ISP protocol (must be first option after devtype)
  -q sets quiet mode (excess messages supressed)
  -r initiates a read (Intel Hex record format)
  -s [size] shows a hash mark status bar of length [size] while erasing/writing
  -w writes to the requested region
     -wpx will suppress actual writing to program space (for debugging picp)
  -v (if given after ttyname or after devtype) show programmer version number
  -v (if only parameter) show picp version number
  Read/Write/Erase parameters:
    p [filename] = program memory, optionally reading/writing filename
    c [val] = configuration bits (val is a numeric word value when writing)
    i [val] = ID locations
    d [filename] = data memory, optionally reading/writing filename
    o [val] = oscillator calibration space
    f = entire flash device (only applies to -e, erase)
  filename is an optional input or output file (default is stdin/stdout)

Flags are operated on in order, from left to right.  If any operation fails,
further execution is aborted.  Thus, a part can be blank checked and programmed
with a single command, e.g.:
        picp /dev/ttyS0 16c505 -bp -wp program.hex 
This example will blank check the program memory of a PIC16C505 then write the
contents of the file program.hex to the program memory only if the blank check
succeeded.
The -wc, -wi, and -wo options must be followed by a numeric argument which
represents the value.  The number may be binary (preceeded by 0b or 0B), hex
(preceeded by 0x or 0X), or decimal (anything else).





http://usbpicprog.org/?page_id=193
https://launchpad.net/~fransschreuder1/+archive/ubuntu/usbpicprog-stable


sudo add-apt-repository ppa:fransschreuder1/usbpicprog-stable
sudo apt update





https://martyn.welchs.me.uk/posts/programming-pic-microcontrollers-under-linux/

sdcc -mpic14 -p16f887 blink.c




http://curuxa.org/en/Program_PICs_with_a_PICkit2_using_the_command_line_on_Linux







https://hackaday.com/2010/11/03/how-to-program-pics-using-linux/
