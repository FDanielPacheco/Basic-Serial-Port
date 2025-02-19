CC   = gcc
CCPP = g++

CFLAGS = -Wall -Wextra -Wpedantic -Wshadow -Wconversion -g -Iinclude
LDFLAGS = -pthread -lcheck_pic -lrt -lm -lsubunit

LIB_NAME = basic_serial
LIB_A = build/c/$(LIB_NAME).a

build/c: src/c/basic_serial.c include/basic_serial.h
		mkdir -p build/c/
		$(CC) $(CFLAGS) -c src/c/basic_serial.c -o build/c/basic_serial.o
		ar rcs $(LIB_A) build/c/basic_serial.o

build/c/example: build/c example/usage_c_api.c include/basic_serial.h
		mkdir -p build/c/example/
		$(CC) $(CFLAGS) example/usage_c_api.c -o build/c/example/usage_c_api.out -Lbuild $(LIB_A) $(LDFLAGS)

install: clean build
		sudo cp include/basic_serial.h /usr/local/include
		sudo cp build/basic_serial.a /usr/local/lib/libbasic_serial.a

clean:
		rm -rf build 

.PHONY: all clean  
all: clean build/c build/c/example