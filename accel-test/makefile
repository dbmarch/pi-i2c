TARGET = accelTest
LIBS = -lm  -lstdc++
CC = gcc
CFLAGS = -g -Wall

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
CPP_OBJECTS = $(patsubst %.cpp, %.o, $(wildcard *.cpp))
HEADERS = $(wildcard *.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS) $(CPP_OBJECTS)

$(TARGET): $(OBJECTS) $(CPP_OBJECTS)
	$(CC) $(OBJECTS) $(CPP_OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
