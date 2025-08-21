#
# Makefile for picp 0.6.7
# PIC programmer interface
#

CC=gcc
APP=picp
INCLUDES=-I.
OPTIONS=-O2 -Wall -x c++
CFLAGS=$(INCLUDES) $(OPTIONS)
SRCS=main.c serial.c picdev.c record.c parse.c atoi_base.c
OBJECTS = main.o serial.o picdev.o record.o parse.o atoi_base.o

WINCC=/usr/local/cross-tools/bin/i386-mingw32msvc-gcc
WINCFLAGS=-Wall -O2 -fomit-frame-pointer -s -I/usr/local/cross-tools/include -D_WIN32 -DWIN32
WINLIBS=
WINOBJECTS = main.obj serial.obj picdev.obj record.obj parse.obj atoi_base.obj

all: $(APP)

$(APP): $(OBJECTS)
	$(CC) $(OBJECTS) -lstdc++ -o $(APP)
	strip $(APP)

clean:
	rm -f *.o
	rm -f $(APP)

install:
	cp -f $(APP) /usr/local/bin/

win: $(APP).exe

$(APP).exe: $(WINOBJECTS)
	$(WINCC) $(WINCFLAGS) $(WINOBJECTS) -o $(APP).exe $(WINLIBS)

main.obj: main.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

serial.obj: serial.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

picdev.obj: picdev.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

record.obj: record.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

parse.obj: parse.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

atoi_base.obj: atoi_base.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

winclean:
	rm -f *.obj
	rm -f $(APP).exe

