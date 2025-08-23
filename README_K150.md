# K150 Programmer Support in PICP

## Overview
PICP now includes support for the K150 PIC programmer, a popular USB-based programmer that uses the Prolific PL2303 USB-to-serial chip.

## Hardware Requirements
- K150 PIC programmer with USB cable
- Prolific PL2303 USB-to-serial adapter (built into K150)
- Target PIC microcontroller

## Usage

### Basic Commands
```bash
# Erase program memory
picp /dev/ttyUSB0 16f628a -ep

# Write program from hex file
picp /dev/ttyUSB0 16f628a -wp program.hex

# Read program memory
picp /dev/ttyUSB0 16f628a -rp program_backup.hex

# Verify program
picp /dev/ttyUSB0 16f628a -vp program.hex
```

### Supported Operations
- **Erase**: `-ep` (program memory), `-ef` (full chip)
- **Write**: `-wp <file>` (program memory), `-wd <file>` (data memory)
- **Read**: `-rp <file>` (program memory), `-rd <file>` (data memory)
- **Verify**: `-vp <file>` (program memory), `-vd <file>` (data memory)

## Hardware Detection
The K150 programmer is automatically detected when connected:
- Uses `/dev/ttyUSB0` by default (Prolific PL2303)
- No CTS signal check required (bypassed for K150)
- LED control via RTS signal for status indication
- DTR signal used for reset pulse

## Troubleshooting

### "K150: Timeout or error waiting for response"
- Check USB cable connection
- Ensure K150 is properly connected to target PIC
- Verify target PIC is inserted correctly in socket
- Check power supply to target circuit

### "programmer not detected (CTS is false)"
This error should not occur with K150 as CTS check is bypassed. If you see this, the K150 detection failed.

### No /dev/ttyUSB0 device
- Check `lsusb` for Prolific Technology PL2303 device
- Install pl2303 kernel module if needed: `sudo modprobe pl2303`
- Check dmesg for USB connection messages

## LED Indicators
- **Yellow LED**: Controlled by RTS signal, blinks during operations
- **Power LED**: Indicates K150 is powered via USB

## Supported PIC Devices
The K150 supports a wide range of PIC microcontrollers. Check your specific model compatibility with the K150 hardware documentation.

## Technical Notes
- Serial communication at 19200 baud, 8N1
- Non-blocking I/O with timeouts
- Automatic port configuration and reset handling
- Compatible with existing PICP command structure
