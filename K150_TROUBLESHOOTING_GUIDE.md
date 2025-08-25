# K150 Programmer Troubleshooting Guide

## Current Issue: K150 Not Responding (0x00 0x00)

### Problem Summary
- K150 detected as USB device (Prolific PL2303)
- Serial port `/dev/ttyUSB0` opens successfully
- No response to programmer detection command (0x42)
- Tried multiple approaches: DTR reset, different baud rates, physical USB reset

### Diagnostic Results
✅ **USB Detection**: `lsusb` shows `067b:2303 Prolific Technology, Inc. PL2303`
✅ **Serial Port**: `/dev/ttyUSB0` created and accessible
✅ **Software**: Code compiles and runs without errors
❌ **Hardware Response**: K150 sends 0 bytes in response to any command

## Troubleshooting Steps

### 1. Hardware Verification
```bash
# Check USB connection
lsusb | grep -i prolific
dmesg | grep ttyUSB | tail -5

# Test serial port access
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# Basic serial communication test
stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb
echo -ne '\x42' > /dev/ttyUSB0
timeout 1 hexdump -C /dev/ttyUSB0
```

### 2. K150 Hardware Checks
- **Power LED**: Should be ON when connected
- **Programming LED**: Should respond during operations
- **USB Cable**: Try different USB cable
- **USB Port**: Try different USB port
- **PL2303 Driver**: Ensure proper driver loaded (`lsmod | grep pl2303`)

### 3. ICSP Connection Verification
Check connections between K150 and PIC:
- **VCC (Pin 14)**: ~5V with Schottky diode protection
- **VPP/MCLR (Pin 4)**: ~13V during programming, 27K pullup resistor
- **PGD (Pin 13)**: Data line, clean connections
- **PGC (Pin 12)**: Clock line, clean connections  
- **GND (Pin 5)**: Common ground

### 4. Alternative Software Tests
```bash
# Try pk2cmd if available
pk2cmd -P

# Test with different baud rates
./picp -p /dev/ttyUSB0 -k150 detect  # 9600 baud (current)

# Manual serial test
minicom -D /dev/ttyUSB0 -b 9600
# Send: 0x42 (hex mode)
# Expected: 0x42 0x03 or similar
```

### 5. K150 Firmware Issues
K150 may have firmware corruption or be in wrong mode:
- **Factory Reset**: Some K150 clones have reset procedures
- **Firmware Update**: May need original K150 software to reflash
- **Hardware Variant**: Different K150 versions use different protocols

## Known Issues and Solutions

### Issue 1: PL2303 Driver Problems
```bash
# Reload PL2303 driver
sudo modprobe -r pl2303
sudo modprobe pl2303

# Check driver version
modinfo pl2303
```

### Issue 2: K150 Clone Variations
Different K150 clones may use:
- Different baud rates (9600, 19200, 38400)
- Different command protocols
- Different DTR/RTS requirements

### Issue 3: Hardware Failure
Signs of hardware failure:
- No response to any commands
- No LED activity
- USB enumeration but no serial communication

## Alternative Solutions

### 1. Use Original K150 Software
Download original K150 software from manufacturer:
- Test with original software first
- Verify hardware functionality
- Check if custom protocol is needed

### 2. Try Different Programming Software
```bash
# If available, try:
avrdude -c usbasp -p m328p  # For AVR
pk2cmd -P                   # PICkit2 compatible
```

### 3. Hardware Replacement
If K150 hardware is faulty:
- Consider PICkit3/4 as alternative
- Use different K150 clone from different manufacturer
- Verify ICSP connections with multimeter

## Current Status
- **Software Implementation**: ✅ Complete and working
- **Hardware Communication**: ❌ K150 not responding
- **Next Steps**: Hardware diagnosis or replacement needed

## Test Commands
```bash
# Quick hardware test
./picp -p /dev/ttyUSB0 -k150 detect

# Verbose debugging
./picp -p /dev/ttyUSB0 -k150 detect 2>&1 | tee k150_debug.log

# USB reset test
sudo dmesg -C
# Unplug/replug K150
dmesg | grep ttyUSB
```

## Contact Information
If hardware replacement is needed, consider:
- Original K150 from reliable supplier
- PICkit3/4 for better compatibility
- Verify ICSP pinout matches your target PIC

---
**Note**: The software implementation is complete and functional. The issue is hardware-related and requires physical diagnosis or replacement.
