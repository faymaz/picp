# K150 LED Control Analysis

## Problem Description
The yellow LED on the K150 programmer stays on after programming operations complete, even though `k150_close_port()` is being called correctly.

## Current Implementation
Our current LED control attempts multiple methods:
1. **RTS Low**: `ioctl(fd, TIOCMBIC, TIOCM_RTS)`
2. **DTR Low**: `ioctl(fd, TIOCMBIC, TIOCM_DTR)`
3. **LED Off Sequence**: Send bytes `{0x00, 0xFF, 0x00}`
4. **Reset Command**: Send byte `0x01`

## K150 Hardware Variations
Different K150 models use different LED control methods:

### Type 1: Standard RTS Control
- LED controlled by RTS signal
- RTS High = LED On, RTS Low = LED Off
- Most common in original K150s

### Type 2: DTR Control
- LED controlled by DTR signal
- Some clones use DTR instead of RTS

### Type 3: Command-Based Control
- LED controlled by specific commands
- Requires sending protocol commands to turn off

### Type 4: Hardware-Only Control
- LED hardwired to power/activity
- Cannot be controlled via software
- Requires physical disconnect

## Debugging Steps

### Step 1: Check Current Signal States
```bash
# Monitor RTS/DTR signals
stty -F /dev/ttyUSB0 -a
```

### Step 2: Test Manual Control
```bash
# Try manual RTS control
stty -F /dev/ttyUSB0 rts
stty -F /dev/ttyUSB0 -rts
```

### Step 3: Protocol Analysis
Monitor what picpro sends to turn off LED:
```bash
# Use serial monitoring tools
sudo cat /dev/ttyUSB0 | hexdump -C
```

## Potential Solutions

### Solution 1: Extended Delay
Some K150s need longer delays for LED control:
```c
ioctl(k150_fd, TIOCMBIC, &rts_flag);
sleep(1);  // 1 second delay instead of 50ms
```

### Solution 2: Power Cycle Sequence
Send power-down sequence:
```c
unsigned char powerdown[] = {0xFF, 0x00, 0xFF, 0x00, 0x00};
write(k150_fd, powerdown, 5);
```

### Solution 3: Multiple Signal Toggle
Toggle RTS/DTR multiple times:
```c
for (int i = 0; i < 3; i++) {
    ioctl(k150_fd, TIOCMBIS, &rts_flag);  // Set high
    usleep(100000);
    ioctl(k150_fd, TIOCMBIC, &rts_flag);  // Set low
    usleep(100000);
}
```

### Solution 4: Hardware Reset
Send hardware reset via break signal:
```c
tcsendbreak(k150_fd, 0);
usleep(500000);
```

## Testing Protocol

1. **Test with standalone utility** (`k150_led_off`)
2. **Monitor serial signals** with oscilloscope/logic analyzer
3. **Compare with picpro behavior** using strace
4. **Test different timing delays**
5. **Try hardware-specific commands**

## Expected Behavior
After successful LED control:
- Yellow LED should turn off immediately
- K150 should enter idle state
- No power consumption from target circuit
- Ready for next programming session

## Fallback Options
If software control fails:
1. **Document the limitation** in user guide
2. **Recommend physical USB disconnect** after operations
3. **Add timeout-based auto-disconnect** feature
4. **Provide manual LED off utility**

---

## Current Status
- ✅ Multiple LED control methods implemented
- ✅ Debug messages added for troubleshooting
- ✅ Standalone LED off utility created
- 🔄 Hardware testing in progress
- ❓ LED still stays on - investigating hardware variant
