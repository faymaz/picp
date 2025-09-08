# PICP - PIC Programmer for Linux

![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=faymaz.picp)

A command-line PIC microcontroller programmer with K150 USB programmer support for Linux.

## Tested Hardware

- **PIC16F628A** - Full read/write/erase support
- **PIC16F876** - Complete backup/restore workflow tested
- **PIC16F887** - 32KB programming support, all operations tested
- **PIC16F84** - Basic functionality verified
- **Data Integrity** - 100% verified with diff comparison

## Features

- **K150 USB Programmer Support** - Reverse-engineered from Microbrn.exe
- **Complete Operations** - Read, Write, Erase, Verify
- **Proper Serial Protocol** - 19200 baud, 8N1, DTR/RTS control
- **Intel HEX Support** - Standard format for firmware files
- **Linux Ready** - Tested on Debian/Ubuntu
- **Production Ready** - Backup/restore workflow validated
- **ZIF Socket Guide** - Pin placement instructions for supported PICs

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

#### Erase chip options
```bash
# Erase program memory only (default)
./picp /dev/ttyUSB0 16f628a -e

# Erase program memory explicitly
./picp /dev/ttyUSB0 16f628a -ep

# Erase full chip (program + data + config)
./picp /dev/ttyUSB0 16f628a -ef
```

#### Chip detection
```bash
./picp /dev/ttyUSB0 -detect
```

#### ZIF Socket Guide
```bash
./picp /dev/ttyUSB0 -zif
```

## Tested Workflow Examples

### PIC16F876 Complete Test
```bash
# 1. Backup original firmware
./picp /dev/ttyUSB0 16f876 -rp backup.hex

# 2. Erase chip  
./picp /dev/ttyUSB0 16f876 -e

# 3. Restore from backup
./picp /dev/ttyUSB0 16f876 -wp backup.hex

# 4. Verify integrity (manual)
./picp /dev/ttyUSB0 16f876 -rp verify.hex
diff backup.hex verify.hex  # Should show no differences
```

### PIC16F628A Programming
```bash
# Program and verify 16F628A
./picp /dev/ttyUSB0 16f628a -wp firmware.hex
```

### PIC16F887 Large Memory Programming
```bash
# 32KB memory programming test
./picp /dev/ttyUSB0 16f887 -rp backup.hex      # 32768 bytes backup
./picp /dev/ttyUSB0 16f887 -e                  # Erase
./picp /dev/ttyUSB0 16f887 -wp backup.hex      # Restore 32KB
./picp /dev/ttyUSB0 16f887 -rp verify.hex      # Verify
diff backup.hex verify.hex                     # Perfect match
```

## Supported PIC Devices

**Tested and Confirmed:**
- PIC16F628A (full read/write/erase)
- PIC16F876 (full backup/restore workflow) 
- PIC16F887 (32KB programming - complete workflow tested)
- PIC16F84 (basic functionality)

**Additional Support:**
- PIC16F series: 627A, 648A, 877A, 72, 73, 74, 76, 77
- PIC18F series: 2550, 4550, 242, 252, 442, 452
- Many classic PIC devices (see `./picp -h` for full list)

**ZIF Socket Placement:**
- PIC16F84/84A: Pin 1 at ZIF pin 2 (18-pin)
- PIC16F628: Pin 1 at ZIF pin 1 (18-pin)
- PIC16F628A: Pin 1 at ZIF pin 2 (18-pin) 
- PIC16F876/877: Pin 1 at ZIF pin 1 (28-pin)
- PIC16F887: Pin 1 at ZIF pin 1 (40-pin)
- PIC12F675/683: ICSP only (8-pin)
- PIC16F690: ICSP only (20-pin)

## Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `-rp file.hex` | Read program memory | `./picp /dev/ttyUSB0 16f628a -rp backup.hex` |
| `-wp file.hex` | Write program memory | `./picp /dev/ttyUSB0 16f628a -wp firmware.hex` |
| `-vp file.hex` | Verify program memory | `./picp /dev/ttyUSB0 16f628a -vp firmware.hex` |
| `-e` | Erase program memory | `./picp /dev/ttyUSB0 16f628a -e` |
| `-ep` | Erase program memory | `./picp /dev/ttyUSB0 16f628a -ep` |
| `-ef` | Erase full chip | `./picp /dev/ttyUSB0 16f628a -ef` |
| `-detect` | Auto-detect chip | `./picp /dev/ttyUSB0 -detect` |
| `-zif` | Show ZIF socket guide | `./picp /dev/ttyUSB0 -zif` |
| `-v` | Verbose mode | `./picp /dev/ttyUSB0 16f628a -v -rp test.hex` |

## Troubleshooting

### Quick Setup Issues
```bash
# 1. Permission fix
sudo usermod -a -G dialout $USER
# Then log out and log back in

# 2. Check USB connection
ls /dev/ttyUSB*

# 3. Test connection
./picp /dev/ttyUSB0 -detect
```

### Common Issues
- **"Permission denied"** - Add user to dialout group (see above)
- **"No such device"** - Check `/dev/ttyUSB*` exists
- **"Bad file descriptor"** - K150 not properly connected/detected
- **Yellow LED stays on** - Communication timeout, check connections

### Debug Mode
```bash
# Use verbose mode for detailed output
./picp /dev/ttyUSB0 16f628a -v -rp test.hex
```

## Technical Details

- **Protocol:** K150 USB (reverse-engineered from Microbrn.exe)
- **Serial:** 19200 baud, 8N1, DTR/RTS control  
- **Commands:** 0x32 programming, 0x14 read, 0x42 init sequence
- **Data:** Intel HEX format for firmware files

## Contributing

```bash
# Build from source
git clone https://github.com/faymaz/picp.git
cd picp
make

# Test with your hardware
./picp /dev/ttyUSB0 -detect
```

Contributions welcome! Test with different PIC models and report results.

## License

- **License:** GNU GPL v2.0
- **Original PICP:** Cosmodog, Ltd. & Jeff Post  
- **K150 Support:** Reverse-engineered protocol implementation (2025)
- **GitHub:** https://github.com/faymaz/picp