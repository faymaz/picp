#
# Makefile for picsnoop
#

APP=picsnoop
CC=gcc

INCLUDES=
OPTIONS=-O2 -Wall -x c++
CFLAGS=$(INCLUDES) $(OPTIONS)
OBJECTS = picsnoop.o serial.o

WINCC=/usr/local/cross-tools/bin/i386-mingw32msvc-gcc
WINCFLAGS=-Wall -O2 -fomit-frame-pointer -s -I/usr/local/cross-tools/include -D_WIN32 -DWIN32
WINLIBS=
WINOBJECTS = picsnoop.obj serial.obj

all : $(APP)

$(APP) : $(OBJECTS)
	$(CC) $(OBJECTS) -lstdc++ -o $(APP)
	strip $(APP)

clean :
	rm -f *.o
	rm -f $(APP)

install:
	cp -f $(APP) /usr/local/bin/
	
win: $(APP).exe

$(APP).exe: $(WINOBJECTS)
	$(WINCC) $(WINCFLAGS) $(WINOBJECTS) -o $(APP).exe $(WINLIBS)

picsnoop.obj: picsnoop.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

serial.obj: serial.c
	$(WINCC) -o $@ $(WINCFLAGS) -c $<

winclean:
	rm -f *.obj
	rm -f $(APP).exe

