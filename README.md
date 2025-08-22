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
- **Automatic K150 Detection**: Plug-and-play USB support
- **PL2303 Compatibility**: Works with standard K150 programmers
- **P18A Protocol**: Reliable communication protocol implementation
- **ICSP Mode**: Support for both ZIF socket and ICSP programming

### New PIC Device Support
- **PIC16F690**: 20-pin, 8K program memory, ICSP compatible
- **PIC18F2550**: 28-pin, 32K program memory, USB-capable MCU
- **Enhanced PIC16C54**: SMD support with ICSP programming
- **PIC16F876/877**: Improved support and compatibility

## 📋 Supported Programmers

| Programmer | Interface | Status | Notes |
|------------|-----------|--------|-------|
| **K150** | USB (PL2303) | 🔄 Partial | Verification and dump improvements in progress |
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

# Program with configuration bits
picp /dev/ttyUSB0 16f84 -wp program.hex -wc 0x3FF4
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
16F84, 16F84A, 16F87, 16F88, 16F627, 16F628, 16F648, 16F676, 16F684, 16F688, **16F690**, 16F72, 16F73, 16F74, 16F76, 16F77, 16F818, 16F819, 16F870, 16F871, 16F872, 16F873, 16F874, 16F876, 16F877, 16F877A

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
- `-wc <value>` - Write configuration bits
- `-rc [file]` - Read configuration bits
- `-wd [file]` - Write data EEPROM
- `-rd [file]` - Read data EEPROM

### Special Modes
- `-i` - Use ICSP protocol (In-Circuit Serial Programming)
- `-v` - Show version information
- `-d` - Show device list or device information
- `-f` - Ignore verify errors
- `-q` - Quiet mode

## 🔍 Troubleshooting

### K150 Issues
- **Device not detected**: Check USB connection and PL2303 drivers
- **Permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Programming fails**: Verify power supply and connections

### General Issues
- **Unknown device**: Check device name spelling and supported device list
- **Verify errors**: Use `-f` flag to ignore or check hardware connections
- **Communication errors**: Try different baud rates or check cable

## 📄 File Formats

### Intel HEX Format
Standard Intel HEX format (.hex files) for program data:
```
:10000000FF3FFF3FFF3FFF3FFF3FFF3FFF3FFF3F70
:10001000FF3FFF3FFF3FFF3FFF3FFF3FFF3FFF3F60
:00000001FF
```

### Configuration Files
- `picdevrc` - Device configuration database
- Contains memory layouts, programming parameters, and device-specific settings

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request

### Adding New Devices
To add support for new PIC devices:
1. Update `picdevrc` with device specifications
2. Add device detection in `main.c`
3. Test with actual hardware
4. Update documentation

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

**PICP 0.6.9** - Making PIC programming accessible with modern USB programmers and enhanced device support.
