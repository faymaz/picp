# PICP - PIC Microcontroller Programmer
![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=faymaz.picp)

**Version 0.6.9** - Enhanced K150 Support Edition

A versatile command-line programmer for Microchip PIC microcontrollers supporting multiple programmer hardware including PICSTART Plus, Warp-13, JuPic, Olimex, and **K150 USB programmers**.

## 🚀 Key Features

- **Multi-Programmer Support**: PICSTART Plus, Warp-13, JuPic, Olimex, and K150 USB
- **163+ Supported PIC Devices**: From PIC10F to PIC18F series
- **K150 USB Support**: Full support for popular K150 USB programmers with PL2303 chipset
- **ICSP Programming**: In-Circuit Serial Programming support
- **Cross-Platform**: Linux, Windows, and other Unix-like systems
- **Intel HEX Format**: Standard hex file support for programming
- **Command-Line Interface**: Scriptable and automation-friendly

## 🆕 What's New in 0.6.9

### Enhanced K150 Support
- **Multiple Protocol Support**: P014, P016, P018, P18A protocols from picpro
- **Robust Serial Communication**: Enhanced timeout handling and error recovery
- **Advanced Fuse Programming**: Both traditional (ON/OFF) and picpro-style (Enabled/Disabled) syntax
- **Automatic Verification**: Read-back confirmation with detailed mismatch reporting
- **Debug Mode**: Verbose output with `-v/--verbose` flag for troubleshooting

### New PIC Device Support
- **PIC16F887**: Full support with comprehensive fuse definitions
- **Enhanced Configuration**: Improved parsing and validation for all device types
- **Verification Dumps**: Detailed mismatch analysis with hex dumps
- **Fallback Mechanisms**: Robust configuration read/write with multiple retry strategies

## 📋 Supported Programmers

| Programmer | Interface | Status | Notes |
|------------|-----------|--------|-------|
| **K150** | USB (PL2303) | ✅ Full Support | Enhanced protocols, verification, and debug support |
| PICSTART Plus | Serial RS232 | ✅ Full Support | Requires firmware 3.00.40+ |
| Warp-13 | Parallel Port | ✅ Supported | Legacy support |
| JuPic | Serial | ✅ Supported | - |
| Olimex | Serial | ✅ Supported | - |

## 🔧 Installation

### Prerequisites
- GCC compiler
- Make utility
- Linux/Unix system with USB support (for K150)

### Build from Source
```bash
git clone https://github.com/faymaz/picp.git
cd picp
make
sudo make install
```

### Testing
```bash
# Run comprehensive test suite
make test

# Test specific fuse combinations
make test-fuses

# Test device support
make test-devices
```

### System Setup
The program requires the device configuration file in the system path:
```bash
sudo cp picdevrc /usr/local/bin/
```

## 📖 Usage Examples

### Basic Programming
```bash
# Program a PIC16F84 using K150
picp /dev/ttyUSB0 16f84 -wp program.hex

# Erase a PIC device
picp /dev/ttyUSB0 16f84 -ep

# Read/backup PIC memory
picp /dev/ttyUSB0 16f84 -rp backup.hex
```

### ICSP Programming
```bash
# Program PIC16F690 via ICSP
picp /dev/ttyUSB0 16f690 -i -wp program.hex

# Program SMD PIC16C54 via ICSP
picp /dev/ttyUSB0 16c54 -i -wp program.hex
```

### Advanced Operations
```bash
# Complete programming sequence
picp /dev/ttyUSB0 16f876 -ep -wp firmware.hex -rp verify.hex

# Program with raw configuration bits
picp /dev/ttyUSB0 -t PIC16F628A -wc 0x3FF4

# Program with named fuse settings (traditional syntax)
picp /dev/ttyUSB0 -t PIC16F628A -wf WDT:ON,CP:OFF,MCLRE:ON

# Program PIC16F887 with picpro-style fuses
picp /dev/ttyUSB0 -t PIC16F887 -wf WDT:Enabled,MCLRE:Enabled,BOREN:Enabled

# Read configuration to file
picp /dev/ttyUSB0 -t PIC16F628A -rc config_backup.hex

# Enable verbose mode for debugging
picp /dev/ttyUSB0 -v -t PIC16F628A -wf WDT:ON,CP:OFF
```

## 🔌 K150 Hardware Setup

### USB Connection
1. Connect K150 programmer to USB port
2. Install PL2303 drivers if needed (usually automatic on Linux)
3. Verify device appears as `/dev/ttyUSB0` (or similar)

### ZIF Socket Programming
- Insert PIC into appropriate ZIF socket (14, 20, or 28-pin)
- Ensure proper orientation (Pin 1 alignment)
- Use standard programming commands

### ICSP Programming
Connect the following pins for ICSP:

| PIC Pin | K150 Pin | Function |
|---------|----------|----------|
| VDD | VDD | Power (+5V) |
| VSS | VSS | Ground |
| PGC | PGC | Programming Clock |
| PGD | PGD | Programming Data |
| MCLR | MCLR | Master Clear/Reset |

## 📊 Supported PIC Devices

### PIC10F Series
10F200, 10F202, 10F204, 10F206

### PIC12F Series  
12F508, 12F509, 12F629, 12F675, 12F683

### PIC16F Series
16F84, 16F84A, 16F87, 16F88, 16F627, 16F628, 16F648, 16F676, 16F684, 16F688, **16F690**, 16F72, 16F73, 16F74, 16F76, 16F77, 16F818, 16F819, 16F870, 16F871, 16F872, 16F873, 16F874, 16F876, 16F877, 16F877A, **16F887**

### PIC16C Series
16C54, 16C54A, 16C54B, 16C54C, 16C55, 16C56, 16C57, 16C58A, 16C61, 16C62, 16C63, 16C64, 16C65, 16C66, 16C67, 16C71, 16C72, 16C73, 16C74, 16C84

### PIC18F Series
**18F2550**, 18F242, 18F252, 18F442, 18F452, 18F458, 18F2410, 18F2431, 18F248, 18F258, 18F4431, 18F448, 18F4550

*And many more... (163+ devices total)*

## 🛠️ Command Reference

### Programming Operations
- `-wp [file]` - Write program memory
- `-rp [file]` - Read program memory  
- `-ep` - Erase program memory
- `-bp` - Blank check program memory

### Configuration & Data
- `-wc <value>` - Write configuration bits (raw hex value)
- `-wf <fuses>` - Write fuse configuration (name:value pairs)
- `-rc [file]` - Read configuration bits to file
- `-wd [file]` - Write data EEPROM
- `-rd [file]` - Read data EEPROM

### Device & Port Options
- `-t <device>` - Specify target device type (e.g., PIC16F628A, PIC16F887)
- `-p <port>` - Specify serial port (default: /dev/ttyUSB0)

### Special Modes
- `-i` - Use ICSP protocol (In-Circuit Serial Programming)
- `-v, --verbose` - Enable verbose/debug output for troubleshooting
- `-d` - Show device list or device information
- `-f` - Ignore verify errors
- `-q` - Quiet mode
- `--dry-run` - Test mode without hardware interaction

## 🔍 Troubleshooting

### K150 Issues
- **Device not detected**: Check USB connection and PL2303 drivers
- **Permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Programming fails**: Verify power supply and connections
- **Communication timeouts**: Use `-v` flag to see detailed debug information

### Debug Mode
```bash
# Enable verbose output to see detailed communication
picp /dev/ttyUSB0 -v -t PIC16F628A -wf WDT:ON

# Check what's happening during programming
picp /dev/ttyUSB0 --verbose -t PIC16F887 -wc 0x3FFF
```

### General Issues
- **Unknown device**: Use `-t` parameter to specify device type explicitly
- **Fuse parsing errors**: Check fuse name spelling and device compatibility
- **Verify errors**: Use `-f` flag to ignore or check hardware connections
- **Communication errors**: Enable verbose mode to diagnose serial issues

## 📄 File Formats

### Intel HEX Format
Standard Intel HEX format (.hex files) for program data:
```
:10000000FF3FFF3FFF3FFF3FFF3FFF3FFF3FFF3F70
:10001000FF3FFF3FFF3FFF3FFF3FFF3FFF3FFF3F60
:00000001FF
```

### Configuration Files
- `picdevrc` - Device configuration database with 163+ PIC devices
- Contains memory layouts, programming parameters, and device-specific settings
- `verification_dump.txt` - Generated during verification failures with detailed mismatch analysis

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request

### Adding New Devices
To add support for new PIC devices:
1. Update `picdevrc` with device specifications
2. Add fuse definitions in `k150_config.c` if needed
3. Test with actual hardware using `make test`
4. Update documentation

### Development
```bash
# Build and test
make clean && make && make test

# Debug with verbose output
./picp /dev/ttyUSB0 -v -t PIC16F628A -wf WDT:ON --dry-run
```

## 📜 License

This project is licensed under the GNU General Public License v2.0 - see the [LICENSE.TXT](LICENSE.TXT) file for details.

## 🙏 Acknowledgments

- Original PICP development by Cosmodog, Ltd. and Jeff Post
- K150 support enhancement and modern device additions
- Community contributions and testing
- Microchip Technology for PIC microcontrollers

## 📞 Support

- **Issues**: Report bugs via GitHub Issues
- **Documentation**: See `PICPmanual.html` for detailed documentation
- **Community**: Join discussions in GitHub Discussions

---

**PICP 0.6.9** - Professional PIC programming with enhanced K150 support, robust error handling, and comprehensive debugging capabilities.
