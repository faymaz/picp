#
# Makefile for picp 0.6.9
# PIC programmer interface with K150 support
#

CC=gcc
APP=picp
INCLUDES=-I.
OPTIONS=-O2 -Wall -x c++
CFLAGS=$(INCLUDES) $(OPTIONS)
SRCS=main.c serial.c record.c parse.c atoi_base.c k150.c
OBJECTS = main.o serial.o record.o parse.o atoi_base.o k150.o

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

install:
	cp -f $(APP) /usr/local/bin/
	cp -f picdevrc /usr/local/bin

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

