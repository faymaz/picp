# Installation Guide - PICP 0.6.9

## Quick Installation

### Linux/Unix Systems

1. **Clone or download the source code**
2. **Build the project:**
   ```bash
   make clean
   make
   ```

3. **Install system-wide (optional):**
   ```bash
   sudo cp picp /usr/local/bin/
   sudo cp picdevrc /usr/local/bin/
   sudo chmod +x /usr/local/bin/picp
   ```

4. **Set up USB permissions for K150:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in for changes to take effect
   ```

## K150 USB Programmer Setup

### Driver Installation
Most Linux distributions automatically detect K150 programmers with PL2303 chipset. If needed:

```bash
# Check if device is detected
lsusb | grep PL2303

# Device should appear as /dev/ttyUSB0 (or similar)
ls /dev/ttyUSB*
```

### Testing K150 Connection
```bash
# Test programmer detection
./picp /dev/ttyUSB0 16f84 -v

# Should output:
# K150 port opened successfully
# K150 programmer detected (firmware type: 3)
# K150 programming support is ready
```

## Troubleshooting

### Permission Issues
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or run with sudo (not recommended)
sudo ./picp /dev/ttyUSB0 16f84 -wp program.hex
```

### Device Not Found
```bash
# Check USB devices
lsusb

# Check serial devices
ls -la /dev/ttyUSB*

# Try different device names
./picp /dev/ttyUSB1 16f84 -v
```

### Build Issues
```bash
# Install build dependencies (Ubuntu/Debian)
sudo apt-get install build-essential gcc make

# Install build dependencies (CentOS/RHEL)
sudo yum groupinstall "Development Tools"
```

## Verification

Test your installation:
```bash
# Show supported devices
./picp -d

# Test K150 detection
./picp /dev/ttyUSB0 16f84 -v

# Program a simple hex file
./picp /dev/ttyUSB0 16f84 -wp yourprogram.hex
```

Installation complete! See README.md for usage examples.
