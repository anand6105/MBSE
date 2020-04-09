RMDIR   := rm -rf
CC      := gcc
BIN     := ./bin
OBJ     := ./obj
INCLUDE := ./inc
SRC     := ./src
SRCS    := $(wildcard $(SRC)/*.c)
OBJS    := $(patsubst $(SRC)/%.c,$(OBJ)/%.o,$(SRCS))
EXE     := $(BIN)/mbse
CFLAGS  := -I$(INCLUDE)
LDLIBS  := -pthread -lrt
MKDIR   := mkdir

.PHONY: all run clean

all: $(EXE)

$(EXE): $(OBJS) | $(BIN)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN) $(OBJ):
	$(MKDIR) $@

run: $(EXE)
	sudo $<

clean:
	$(RMDIR) $(OBJ) $(BIN)
