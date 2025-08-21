#
# Picsnoop makefile for Borland C++
#

CC = bcc
CFLAGS = -ml -c
LINK = tlink
OBJS = picsnoop.obj serial.obj
TARGET = picsnoop.exe

$(TARGET):	$(OBJS)
	$(LINK)	/i c0l $(OBJS),$(TARGET),,cl

picsnoop.obj:	picsnoop.c serial.h
	$(CC)	$(CFLAGS) $*.c

serial.obj:	serial.c serial.h
	$(CC)	$(CFLAGS) $*.c

clean:
	del *.obj
	del $(TARGET)

# end of file

