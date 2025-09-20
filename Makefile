# Simple Makefile for picp K150 testing
CC = gcc
CFLAGS = -Wall -O2 -g -DDEBUG
LDFLAGS = -lm

# Essential source files only
SOURCES = main.c k150.c k150_config.c picdev.c serial.c debug.c parse.c verify.c atoi_base.c record.c

OBJECTS = $(SOURCES:.c=.o)
TARGET = picp

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all clean
