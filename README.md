# PICP - PIC Programmer

A command-line PIC microcontroller programmer supporting K150 and other programmers.

## Features

- K150 programmer support
- PIC16F and PIC18F device support
- ROM programming, reading, and verification
- Configuration bit (fuse) programming
- Erase operations

## Usage

```bash
# Program a PIC device
./picp -wp program.hex PIC16F628A

# Read ROM from device
./picp -rp backup.hex PIC16F628A

# Erase device
./picp -ep PIC16F628A

# Program configuration bits
./picp -wf WDT:ON,CP:OFF PIC16F628A
```

## Compilation

```bash
make clean
make
```

## Hardware Support

- K150 USB programmer via /dev/ttyUSB0
- 164+ PIC devices supported
- Serial communication at 19200 baud

## License

See LICENSE.TXT for details.
