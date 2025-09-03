# PICP - PIC Programmer for Linux

![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=faymaz.picp)

A command-line PIC microcontroller programmer with **K150 USB programmer support** for Linux.

## Features

- 🚀 **K150 USB Programmer Support** - Full protocol implementation
- 📱 **Wide Device Support** - PIC16F628A, PIC16F84A, PIC16F876A, PIC16F887, PIC18F2550 and more
- 💾 **Complete Operations** - Read, Write, Erase, Verify, Configuration programming
- 🔧 **Fuse Configuration** - Human-readable fuse programming (WDT:ON, CP:OFF, etc.)
- 📡 **19200 baud, 8N1** - Proper serial settings  
- 🔌 **DTR/RTS Control** - Hardware handshaking
- 🐧 **Linux Ready** - Debian/Ubuntu tested
- 📋 **ROM Operations** - Read/write/verify program memory

## Quick Start

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install build-essential gcc make
```

### 2. Build PICP
```bash
git clone https://github.com/faymaz/picp.git
cd picp
make
```

### 3. Connect K150 Programmer
```bash
# Check USB connection
ls /dev/ttyUSB*

# Add user to dialout group (required for serial access)
sudo usermod -a -G dialout $USER
# Log out and log back in for group changes to take effect
```

### 4. Basic Usage Examples

#### Read PIC ROM to HEX file
```bash
./picp /dev/ttyUSB0 16f628a -rp firmware.hex
```

#### Program PIC from HEX file
```bash
./picp /dev/ttyUSB0 16f628a -wp firmware.hex
```

#### Verify programmed data
```bash
./picp /dev/ttyUSB0 16f628a -vp firmware.hex
```

#### Program with fuse configuration
```bash
./picp /dev/ttyUSB0 16f628a -wp firmware.hex -wf WDT:ON,CP:OFF,MCLRE:ON
```

## K150 Advanced Features

### Chip Detection
```bash
./picp /dev/ttyUSB0 -detect
```

### Fuse Programming (Human Readable)
```bash
# PIC16F628A
./picp /dev/ttyUSB0 -t 16f628a -wf WDT:ON,CP:OFF,PWRT:OFF,MCLRE:ON,BODEN:ON,LVP:ON,CPD:OFF

# PIC16F887
./picp /dev/ttyUSB0 -t 16f887 -wf WDT:Enabled,PWRTE:Disabled,MCLRE:Enabled,BOREN:Enabled

# PIC18F2550
./picp /dev/ttyUSB0 -t 18f2550 -wf WDT:ON,LVP:ON,MCLRE:ON,CP0:OFF,CP1:OFF,CPB:OFF,CPD:OFF
```

### Raw Configuration Programming
```bash
./picp /dev/ttyUSB0 -t 16f628a -wc 0x3FF4
```

### Configuration Reading
```bash
./picp /dev/ttyUSB0 -t 16f628a -rc config.hex
```

### Complete Programming Workflow
```bash
# 1. Detect chip
./picp /dev/ttyUSB0 -detect

# 2. Erase chip
./picp /dev/ttyUSB0 16f628a -ep

# 3. Program firmware
./picp /dev/ttyUSB0 16f628a -wp firmware.hex

# 4. Set fuses
./picp /dev/ttyUSB0 -t 16f628a -wf WDT:ON,CP:OFF,MCLRE:ON

# 5. Verify programming
./picp /dev/ttyUSB0 16f628a -vp firmware.hex
```

## Supported PIC Devices

### PIC16F Series
- PIC16F628A, PIC16F84A, PIC16F876A, PIC16F887
- PIC16F627A, PIC16F648A, PIC16F877A
- PIC16F72, PIC16F73, PIC16F74, PIC16F76, PIC16F77

### PIC18F Series  
- PIC18F2550, PIC18F4550
- PIC18F242, PIC18F252, PIC18F442, PIC18F452

### And many more classic PIC devices...

## Command Reference

### Basic Operations
| Command | Description | Example |
|---------|-------------|---------|
| `-rp file.hex` | Read program memory | `./picp /dev/ttyUSB0 16f628a -rp backup.hex` |
| `-wp file.hex` | Write program memory | `./picp /dev/ttyUSB0 16f628a -wp firmware.hex` |
| `-vp file.hex` | Verify program memory | `./picp /dev/ttyUSB0 16f628a -vp firmware.hex` |
| `-vd file.hex` | Verify data memory | `./picp /dev/ttyUSB0 16f628a -vd eeprom.hex` |
| `-ep` | Erase program memory | `./picp /dev/ttyUSB0 16f628a -ep` |

### K150 Specific Commands
| Command | Description | Example |
|---------|-------------|---------|
| `-detect` | Auto-detect chip type | `./picp /dev/ttyUSB0 -detect` |
| `-t device` | Specify device type | `./picp /dev/ttyUSB0 -t 16f628a` |
| `-wf fuses` | Write fuses (human readable) | `./picp /dev/ttyUSB0 -t 16f628a -wf WDT:ON,CP:OFF` |
| `-wc value` | Write config (raw hex) | `./picp /dev/ttyUSB0 -t 16f628a -wc 0x3FF4` |
| `-rc file.hex` | Read config to file | `./picp /dev/ttyUSB0 -t 16f628a -rc config.hex` |

### Debug Options
| Command | Description |
|---------|-------------|
| `-v` | Verbose/debug mode |
| `-h` | Show help |
| `-d` | Show device list |

## Troubleshooting

### Permission Issues
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or run with sudo (not recommended)
sudo ./picp /dev/ttyUSB0 16f628a -rp firmware.hex
```

### USB Device Not Found
```bash
# Check USB devices
lsusb | grep -i "ch340\|cp210\|ftdi"

# Check serial devices
ls -la /dev/ttyUSB*

# Check dmesg for device recognition
dmesg | tail -20
```

### K150 Not Detected
```bash
# Test with verbose mode
./picp /dev/ttyUSB0 16f628a -v

# Try different USB ports
ls /dev/ttyUSB*

# Check K150 LED status (should be green when connected)
```

For K150 issues:
- Check USB connection (`ls /dev/ttyUSB*`)
- Verify permissions (`groups $USER`)  
- Enable debug mode (`-v` flag)
- Test with known-good PIC chip

## Technical Details

### K150 Protocol
- **Baud Rate:** 19200
- **Data Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow Control:** Manual DTR/RTS control
- **Protocol:** Enhanced P018 + Microbrn.exe compatible sequences
- **Auto-Response:** 0x42 0x03 0x42 sequence support

### Supported Fuse Options

#### PIC16F628A
- `WDT` - Watchdog Timer (ON/OFF)
- `CP` - Code Protection (ON/OFF)  
- `PWRT` - Power-up Timer (ON/OFF)
- `MCLRE` - Master Clear Enable (ON/OFF)
- `BODEN` - Brown-out Detect (ON/OFF)
- `LVP` - Low Voltage Programming (ON/OFF)
- `CPD` - Data Code Protection (ON/OFF)

#### PIC16F887
- `WDT` - Watchdog Timer (Enabled/Disabled)
- `PWRTE` - Power-up Timer (Enabled/Disabled)
- `MCLRE` - Master Clear (Enabled/Disabled)
- `BOREN` - Brown-out Reset (Enabled/Disabled)
- `LVP` - Low Voltage Programming (Enabled/Disabled)

## Development

### Building from Source
```bash
git clone https://github.com/faymaz/picp.git
cd picp
make clean
make
```

### Contributing
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with real hardware
5. Submit a pull request

### Testing
```bash
# Run built-in tests
make test

# Test specific device
make test-devices

# Test fuse parsing
make test-fuses
```

## License

GNU General Public License v2.0

## Authors

- Original PICP: Cosmodog, Ltd. (2000-2004) & Jeff Post (2004-2006)
- K150 Support: Enhanced implementation (2025-2026)

## Links

- **GitHub:** https://github.com/faymaz/picp
- **Issues:** https://github.com/faymaz/picp/issues
- **K150 Protocol:** Based on reverse-engineered Microbrn.exe communication

---

**⚡ Ready to program your PICs with K150 on Linux!** 🐧🔧