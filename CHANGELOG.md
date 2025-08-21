# PICP Changelog

## Version 0.6.9 (21 August 2025)

### New Features
- **K150 Programmer Support**: Added full support for K150 USB PIC programmer
  - Automatic K150 detection on `/dev/ttyUSB0`
  - Compatible with PL2303-based K150 programmers
  - Implements P18A protocol for reliable communication
  - Supports both ICSP and socket programming modes

### Device Support Added
- **PIC16F887**: 8K program memory, 256 bytes data memory, K150 compatible
- **PIC18F2550**: 32K program memory, 256 bytes data memory, K150 compatible

### Technical Improvements
- Enhanced programmer detection logic with K150 priority
- Updated device definitions with K150 programmer bitmap flags
- Improved error handling for K150 communication
- Added comprehensive K150 protocol implementation

### Files Modified
- `main.c`: Added K150 detection and support logic
- `picdev.c`: Added PIC16F887 and PIC18F2550 device definitions
- `picdev.h`: Added P_K150 programmer constant
- `Makefile`: Updated to include K150 module compilation
- `k150.c`: New K150 protocol implementation
- `k150.h`: New K150 function declarations

### Compatibility
- Maintains full backward compatibility with existing programmers:
  - PICSTART Plus
  - Warp-13
  - JuPic
  - Olimex
- K150 support works alongside existing programmer detection

### Usage Examples
```bash
# Program PIC16F887 with K150
./picp /dev/ttyUSB0 PIC16F887 -wp program.hex

# Backup PIC18F2550 ROM
./picp /dev/ttyUSB0 PIC18F2550 -rp backup.hex

# Erase PIC16F887
./picp /dev/ttyUSB0 PIC16F887 -ep
```

### Build Requirements
- GCC compiler with C++ support
- Standard Linux development tools
- K150 programmer connected via USB

---

## Previous Versions
See HISTORY file for versions 0.6.8 and earlier.
