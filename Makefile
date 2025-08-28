#
# Makefile for picp 0.6.9
# PIC programmer interface with K150 support
#

CC=gcc
APP=picp
INCLUDES=-I.
OPTIONS=-O2 -Wall
CFLAGS=$(INCLUDES) $(OPTIONS)
SRCS=main.c serial.c record.c parse.c atoi_base.c k150.c k150_config.c debug.c
OBJS = main.o parse.o record.o atoi_base.o picdev.o serial.o k150.o k150_config.o debug.o

WINCC=/usr/local/cross-tools/bin/i386-mingw32msvc-gcc
WINCFLAGS=-Wall -O2 -fomit-frame-pointer -s -I/usr/local/cross-tools/include -D_WIN32 -DWIN32
WINLIBS=
WINOBJECTS = main.obj serial.obj record.obj parse.obj atoi_base.obj k150.obj

all: $(APP) convert convertshort

$(APP): $(OBJS)
	$(CC) $(OBJS) -lstdc++ -o $(APP)

convert: convert.c
	$(CC) -O2 -Wall -o convert convert.c
	strip convert

convertshort: convertshort.c
	$(CC) -O2 -Wall -o convertshort convertshort.c
	strip convertshort

clean:
	rm -f *.o
	rm -f $(APP)
	rm -f convert
	rm -f convertshort

uninstall:
	@echo "Uninstalling PICP..."
	rm -f /usr/local/bin/$(APP)
	rm -f /usr/local/bin/picdevrc
	rm -f /usr/local/bin/convert
	rm -f /usr/local/bin/convertshort
	@echo "✓ PICP uninstalled"

install: $(APP) convert convertshort
	@echo "Installing PICP..."
	@if [ "$$(id -u)" != "0" ]; then \
		echo "Error: Installation requires root privileges. Use 'sudo make install'"; \
		exit 1; \
	fi
	install -d /usr/local/bin
	install -m 755 $(APP) /usr/local/bin/
	install -m 644 picdevrc /usr/local/bin/
	install -m 755 convert /usr/local/bin/
	install -m 755 convertshort /usr/local/bin/
	@echo "✓ PICP installed successfully!"
	@echo "✓ Binary: /usr/local/bin/$(APP)"
	@echo "✓ Data file: /usr/local/bin/picdevrc"
	@echo "✓ Utilities: convert, convertshort"
	@echo ""
# @echo "Usage: picp /dev/ttyUSB0 16f628a -ep"

win: $(APP).exe convert.exe convertshort.exe

$(APP).exe: $(WINOBJECTS)
	$(WINCC) $(WINCFLAGS) $(WINOBJECTS) -o $(APP).exe $(WINLIBS)

main.obj: main.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

serial.obj: serial.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

record.obj: record.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

parse.obj: parse.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

atoi_base.obj: atoi_base.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

k150.obj: k150.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

convert.exe: convert.c
	$(WINCC) -o $@ $(WINCFLAGS) $<

convertshort.exe: convertshort.c
	$(WINCC) -o $@ $(WINCFLAGS) $<

winclean:
	rm -f *.obj
	rm -f $(APP).exe
	rm -f convert.exe
	rm -f convertshort.exe

# Test targets
test: $(APP)
	@echo "Running PICP K150 tests..."
	@echo "Testing fuse parsing..."
	./$(APP) /dev/ttyUSB0 -t PIC16F628A -wf WDT:ON,CP:OFF --dry-run || echo "Fuse test failed"
	@echo "Testing PIC16F887 support..."
	./$(APP) /dev/ttyUSB0 -t PIC16F887 -wf WDT:Enabled,MCLRE:Enabled --dry-run || echo "PIC16F887 test failed"
	@echo "Testing raw config..."
	./$(APP) /dev/ttyUSB0 -t PIC16F628A -wc 0x3FF4 --dry-run || echo "Raw config test failed"
	@echo "Testing verbose mode..."
	./$(APP) /dev/ttyUSB0 -v -t PIC16F628A -wf WDT:ON --dry-run | grep -q "DEBUG:" && echo "Verbose test passed" || echo "Verbose test failed"
	@echo "✓ All tests completed"

test-fuses: $(APP)
	@echo "Testing all supported fuse combinations..."
	./$(APP) -t PIC16F628A -wf CP:OFF,WDT:ON,PWRT:OFF,MCLRE:ON,BODEN:ON,LVP:ON,CPD:OFF --dry-run
	./$(APP) -t PIC16F887 -wf CP:OFF,WDT:Enabled,PWRTE:Disabled,MCLRE:Enabled,BOREN:Enabled,LVP:Enabled,CPD:Disabled --dry-run
	./$(APP) -t PIC18F2550 -wf WDT:ON,LVP:ON,MCLRE:ON,CP0:OFF,CP1:OFF,CPB:OFF,CPD:OFF --dry-run
	@echo "✓ Fuse tests completed"

test-devices: $(APP)
	@echo "Testing device support..."
	@for device in PIC16F628A PIC16F84A PIC16F876A PIC16F887 PIC18F2550; do \
		echo "Testing $$device..."; \
		./$(APP) -t $$device -wf WDT:ON --dry-run || echo "$$device test failed"; \
	done
	@echo "✓ Device tests completed"

.PHONY: all clean install uninstall win winclean test test-fuses test-devices
