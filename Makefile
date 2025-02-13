CC = gcc
CFLAGS = -Wall -Wextra -g -Iinclude 
LDFLAGS = 

LIB_NAME = basic_serial
LIB_A = build/$(LIB_NAME).a

$(LIB_A): src/*.c include/basic_serial.h
		mkdir -p build
		$(CC) $(CFLAGS) -c src/*.c -o build/*.o
		ar rcs $(LIB_A) build/*.o

bin/example/1: example/1.c $(LIB_A) include/basic_serial.h
		mkdir -p bin/example
		$(CC) $(CFLAGS) example/1.c -o bin/example/1.out -Lbuild $(LIB_A) $(LDFLAGS)

clean:
		rm -rf build bin 

.PHONY: all clean  
all: $(LIB_A) build/example/1