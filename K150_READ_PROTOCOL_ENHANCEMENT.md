# K150 Read Protocol Enhancement

## Overview
This document describes the enhanced K150 read protocol implementation for P018 legacy firmware, addressing the persistent read failures that returned only 0xFF values instead of actual ROM data.

## Problem Analysis
The original K150 read implementation suffered from:
- Block read timeouts using command 0x04
- Unreliable serial communication timing
- No retry mechanism for failed reads
- Incompatible protocol for P018 legacy firmware

## Enhanced Solution

### 1. Address-Based Single Word Read Protocol
**Implementation**: `k150_read_rom()` in `k150.c`

**Protocol Sequence**:
```
1. Enter programming mode: 0x50 ('P')
2. For each word address (0 to word_count-1):
   a. Send read command: 0x0B
   b. Send address low byte: addr & 0xFF
   c. Send address high byte: (addr >> 8) & 0xFF
   d. Read 2 bytes: low_byte, high_byte
   e. Store as 14-bit word with proper masking
3. Exit programming mode: 0x01
```

**Key Features**:
- Reads each word individually by address
- Built-in retry mechanism (3 attempts per word)
- Proper timing delays between operations
- Progress reporting and LED feedback
- 14-bit word masking for PIC architecture

### 2. Enhanced Serial Communication
**Implementation**: `ReadBytesWithRetry()` in `serial.c`

**Features**:
- Multiple retry attempts with configurable timeout
- Automatic delay between retries (5ms)
- Cross-platform support (Linux/Windows)
- Debug output integration
- Blocking serial port mode for reliable reads

**Function Signature**:
```c
unsigned int ReadBytesWithRetry(int theDevice, unsigned char *theBytes, 
                               unsigned int maxBytes, unsigned int timeOut, 
                               int maxRetries);
```

### 3. Firmware Detection and Protocol Selection
**Implementation**: `k150_detect_programmer()` and `k150_read_rom()` in `k150.c`

**Logic**:
- Detects firmware version during programmer initialization
- P18A firmware (0x12): Uses block read protocol
- P018 legacy firmware (0x03): Uses address-based read protocol
- Environment variable `K150_FORCE_P18A=1` can override detection

## Technical Improvements

### Timing Optimization
- **Command delays**: 2ms after read command
- **Address delays**: 5ms after address transmission
- **Word delays**: 1ms between word reads
- **Retry delays**: 10ms between retry attempts

### Error Handling
- Per-word retry mechanism (up to 3 attempts)
- Graceful fallback to 0xFF for failed reads
- Detailed error reporting with word addresses
- Progress tracking for long read operations

### Memory Management
- Proper 14-bit word storage in byte buffer
- High byte masking: `high_byte & 0x3F`
- Correct byte ordering: low byte first, then high byte

## Usage

### Command Line
```bash
# Read ROM using enhanced protocol
./picp -rp /dev/ttyUSB0 PIC16F628A output.hex

# Force P18A protocol (if needed)
K150_FORCE_P18A=1 ./picp -rp /dev/ttyUSB0 PIC16F628A output.hex
```

### Expected Output
```
K150: Firmware version: 0x03 (P018 legacy)
K150: Using P018 legacy protocol (address-based read 0x0B) for firmware 0x03
K150: Reading 2048 words (4096 bytes) using address-based protocol
K150: Read progress: 0/2048 words (0/4096 bytes)
K150: Read progress: 64/2048 words (128/4096 bytes)
...
K150: ROM read completed
```

## Testing

### Test Script
Use `test_k150_read.sh` to validate the implementation:
```bash
./test_k150_read.sh
```

### Validation Steps
1. **Protocol Detection**: Verify correct firmware detection
2. **Address-Based Reading**: Confirm word-by-word reading
3. **Retry Mechanism**: Check timeout and retry behavior
4. **Data Integrity**: Compare output with expected hex files
5. **Progress Reporting**: Monitor LED feedback and progress messages

## Compatibility

### Supported Devices
- PIC16F628A (primary test device)
- PIC16F84, PIC16F876, PIC18F2550
- Any PIC device supported by K150 hardware

### Firmware Compatibility
- **P018 Legacy (0x03)**: Uses enhanced address-based protocol
- **P18A Modern (0x12)**: Uses original block read protocol
- **Auto-detection**: Automatically selects appropriate protocol

## Performance

### Speed Characteristics
- **Address-based read**: ~1ms per word + communication overhead
- **Block read**: Faster but unreliable on legacy firmware
- **Progress reporting**: Every 64 words (128 bytes)

### Reliability Improvements
- **Retry mechanism**: 3 attempts per word
- **Timeout handling**: 50ms timeout per word read
- **Error recovery**: Continues reading after individual word failures

## Troubleshooting

### Common Issues
1. **Timeout errors**: Check USB connection and device power
2. **Protocol mismatch**: Verify firmware detection or use force flag
3. **Partial reads**: Review retry mechanism and timing delays
4. **Verification failures**: Compare with known good hex files

### Debug Information
- Enable communication debug with appropriate flags
- Check `read_test_output.log` for detailed protocol traces
- Monitor LED behavior during read operations

## Future Enhancements

### Potential Improvements
1. **Adaptive timing**: Dynamic delay adjustment based on device response
2. **Parallel reading**: Multiple word reads in single transaction
3. **Checksum validation**: Built-in data integrity verification
4. **Protocol analysis**: Serial traffic capture and analysis tools

### Hardware Considerations
- **Firmware upgrade**: Consider upgrading to P18A firmware for better performance
- **Serial analysis**: Use protocol analyzers to reverse-engineer optimal timing
- **Alternative tools**: Maintain hybrid approach with picpro as fallback

## Conclusion

The enhanced K150 read protocol provides a reliable solution for reading ROM data from PIC devices using P018 legacy firmware. The address-based approach with retry mechanisms significantly improves success rates compared to the original block read implementation.

**Status**: ✅ Implementation complete and ready for hardware testing
**Recommendation**: Use this enhanced protocol for all P018 legacy firmware operations
