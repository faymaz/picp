#
# Makefile for picp 0.6.9
# PIC programmer interface with K150 support
#

CC=gcc
APP=picp
INCLUDES=-I.
OPTIONS=-O2 -Wall
CFLAGS=$(INCLUDES) $(OPTIONS)
SRCS=main.c serial.c record.c parse.c atoi_base.c k150.c
OBJECTS = main.o serial.o record.o parse.o atoi_base.o k150.o verify.o

WINCC=/usr/local/cross-tools/bin/i386-mingw32msvc-gcc
WINCFLAGS=-Wall -O2 -fomit-frame-pointer -s -I/usr/local/cross-tools/include -D_WIN32 -DWIN32
WINLIBS=
WINOBJECTS = main.obj serial.obj record.obj parse.obj atoi_base.obj k150.obj

all: $(APP) convert convertshort

$(APP): $(OBJECTS)
	$(CC) $(OBJECTS) -lstdc++ -o $(APP)
	strip $(APP)

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

.PHONY: all clean install uninstall win winclean
