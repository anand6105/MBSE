PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

HEADERS = -Iinc/
CC = gcc
OBJS = main.o
CFLAGS += -g
LDFLAGS += -pthread

main:	$(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o:	$(PROJECT_ROOT)src/*.c
	$(CC) $(HEADERS) -c -Wall $(CFLAGS) -o $@ $<

clean:
	rm -fr main $(OBJS)
