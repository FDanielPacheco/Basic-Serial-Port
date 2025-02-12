# Makefile for LibLinux-Serial_C

CC = gcc
CFLAGS = -Wall -Wextra -g -Iinclude
LDFLAGS = 

build/basic_serial.a: src/*.c include/basic_serial.h
		mkdir -p build
		$(CC) $(CFLAGS) -c src/*.c -o build/*.o
		ar rcs build/basic_serial.a build/*.o

bin/test: src/*.c include/basic_serial.h
		mkdir -p bin
		$(CC) $(CFLAGS) -o bin/basic_serial src/*.c


#build/example: example/example.c build/libserial.a include/serial.h
#        $(CC) $(CFLAGS) example/example.c -o build/example -Lbuild -lserial $(LDFLAGS) # Compiler will search include/

# ... (rest of your Makefile)