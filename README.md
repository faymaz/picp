# PICP - PIC Programmer for Linux

![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=faymaz.picp)

A command-line PIC microcontroller programmer with **K150 USB programmer support** for Linux.

## ✨ K150 Support 

**Full K150 USB programmer support** - Compatible with Microbrn.exe protocol and works on Linux without additional drivers.

### Tested Devices
- ✅ **PIC16F628A** - Fully working (ROM read/write)  
- ✅ **PIC16F84A** - Compatible
- ✅ **K150 USB Programmer** - Protocol reverse-engineered

### K150 Features
- 🔧 **Microbrn.exe Compatible** - Uses same communication protocol
- 📡 **19200 baud, 8N1** - Proper serial settings  
- 🔌 **DTR/RTS Control** - Hardware handshaking
- 🐧 **Linux Ready** - Debian/Ubuntu tested
- 📋 **ROM Operations** - Read/write/verify program memory

## Quick Start

### K150 Usage
```bash
# Read ROM from PIC16F628A
./picp /dev/ttyUSB0 16f628a -rp output.hex

# Write ROM to PIC16F628A  
./picp /dev/ttyUSB0 16f628a -wp input.hex

# Read configuration bits
./picp /dev/ttyUSB0 16f628a -rc

# Erase chip
./picp /dev/ttyUSB0 16f628a -ef
```

### Device Detection
```bash
# Check if K150 is connected
./picp /dev/ttyUSB0 16f628a -v

# List supported devices
./picp -d
```

## Installation

### Build from Source
```bash
git clone https://github.com/faymaz/picp.git
cd picp
make
```

### Dependencies
- GCC compiler
- Linux kernel (for USB serial support)
- Permission to access `/dev/ttyUSB*` devices

### USB Permissions
```bash
# Add user to dialout group for USB access
sudo usermod -a -G dialout $USER
# or set temporary permissions  
sudo chmod 666 /dev/ttyUSB0
```

## Supported Programmers

### K150 USB Programmer ⭐ 
- **Status**: Fully supported
- **Protocol**: P018/Microbrn.exe compatible  
- **Connection**: USB to serial (PL2303 chipset)
- **Tested**: PIC16F628A, PIC16F84A

### Other Programmers
- PICStart Plus  
- Warp-13
- JuPic
- Tait classic
- AN589 (experimental)

## Supported PIC Devices

### PIC16F Series
- PIC16F84A, PIC16F628A, PIC16F648A
- PIC16F72, PIC16F73, PIC16F74, PIC16F76, PIC16F77
- PIC16F870, PIC16F871, PIC16F872, PIC16F873, PIC16F874
- PIC16F876, PIC16F877, PIC16F876A, PIC16F877A
- And many more...

### PIC18F Series  
- PIC18F242, PIC18F252, PIC18F442, PIC18F452
- PIC18F248, PIC18F258, PIC18F448, PIC18F458
- And others...

## Command Line Options

```
Usage: picp [device] [PIC] [options]

Device: 
  /dev/ttyUSB0    K150 USB programmer
  /dev/ttyS0      Serial port programmer
  
PIC:
  16f628a         PIC16F628A
  16f84a          PIC16F84A  
  (see -d for full list)

Options:
  -rp file.hex    Read program memory to Intel HEX file
  -wp file.hex    Write program memory from Intel HEX file  
  -rc             Read configuration bits
  -wc value       Write configuration bits
  -ef             Erase flash memory
  -v              Verbose output
  -d              List supported devices
```

## Development

### K150 Protocol Implementation
The K150 support was reverse-engineered from Microbrn.exe using serial port monitoring:

1. **DTR/RTS Handshake** - Wake up sequence
2. **Auto-response Detection** - `42 03 42` signature  
3. **P018 Protocol Init** - `50 03` command
4. **Device Parameters** - Configuration sequence
5. **ROM Commands** - `14` for read operations

See `k150.c` for full implementation details.

### Contributing
- Fork the repository
- Create feature branch
- Test with real hardware
- Submit pull request

## License

GNU General Public License v2.0

## Credits

- **Original Author**: Jeff Post (2004-2006)  
- **Linux Port**: Cosmodog, Ltd. (2000-2004)
- **K150 Support**: Faymaz (2025)
- **Protocol Analysis**: Based on Microbrn.exe behavior

## Support

For K150 issues:
- Check USB connection (`ls /dev/ttyUSB*`)
- Verify permissions (`groups $USER`)  
- Enable debug mode (`-v` flag)
- Test with known-good PIC chip

Hardware tested:
- K150 USB Programmer (PL2303 chipset)
- PIC16F628A in ZIF socket
- Debian 12, Ubuntu 22.04