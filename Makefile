CC = gcc
CFLAGS = -Wall -Wextra -g -Iinclude
LDFLAGS = -pthread -lcheck_pic -lrt -lm -lsubunit

LIB_NAME = basic_serial
LIB_A = build/$(LIB_NAME).a

build: src/*.c include/basic_serial.h
		mkdir -p build
		$(CC) $(CFLAGS) -c src/*.c -o build/*.o
		ar rcs $(LIB_A) build/*.o

bin/example/1: example/1.c $(LIB_A) include/basic_serial.h
		mkdir -p bin/example
		$(CC) $(CFLAGS) example/1.c -o bin/example/1.out -Lbuild $(LIB_A) $(LDFLAGS)

bin/test: test/unit.c $(LIB_A) include/basic_serial.h
		mkdir -p bin/test
		$(CC) $(CFLAGS) test/unit.c -o bin/test/unit.out -Lbuild $(LIB_A) $(LDFLAGS)

install: clean build
		sudo cp include/basic_serial.h /usr/local/include
		sudo cp build/basic_serial.a /usr/local/lib/libbasic_serial.a

clean:
		rm -rf build bin 

.PHONY: all clean  
all: $(LIB_A) build/example/1