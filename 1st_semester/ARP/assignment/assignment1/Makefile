# Compiler, source, include, binary, and logs directory settings
CC   = gcc
SRC  = src
INC  = include
BIN  = bin

.PHONY: all clean logs

# The 'all' target compiles all executables
all: logs $(BIN) main server drone watchdog keyboard_manager map_window obstacle target

# Create the bin directory if it does not exist
$(BIN):
	mkdir -p $(BIN)

logs:
	mkdir -p logs

main: $(SRC)/main.c
	$(CC) -I$(INC) -o $(BIN)/main $(SRC)/main.c -lcjson -lrt -pthread && \
		echo "Compilation of main.c completed successfully\n" || \
		echo "Error compiling main.c\n"

server: $(SRC)/server.c
	$(CC) -I$(INC) -o $(BIN)/server $(SRC)/server.c -lrt -pthread && \
		echo "Compilation of server.c completed successfully\n" || \
		echo "Error compiling server.c\n"

drone: $(SRC)/drone.c
	$(CC) -I$(INC) -o $(BIN)/drone $(SRC)/drone.c -lm -lrt -pthread && \
		echo "Compilation of drone.c completed successfully\n" || \
		echo "Error compiling drone.c\n"

watchdog: $(SRC)/watchdog.c
	$(CC) -I$(INC) -o $(BIN)/watchdog $(SRC)/watchdog.c -lrt -pthread && \
		echo "Compilation of watchdog.c completed successfully\n" || \
		echo "Error compiling watchdog.c\n"

keyboard_manager: $(SRC)/keyboard_manager.c
	$(CC) -I$(INC) -o $(BIN)/keyboard_manager $(SRC)/keyboard_manager.c -lncurses -lrt -pthread && \
		echo "Compilation of keyboard_manager.c completed successfully\n" || \
		echo "Error compiling keyboard_manager.c\n"

map_window: $(SRC)/map_window.c
	$(CC) -I$(INC) -o $(BIN)/map_window $(SRC)/map_window.c -lncurses -lrt -pthread && \
		echo "Compilation of map_window.c completed successfully\n" || \
		echo "Error compiling map_window.c\n"

obstacle: $(SRC)/obstacle.c
	$(CC) -I$(INC) -o $(BIN)/obstacle $(SRC)/obstacle.c -lrt -pthread -lbsd && \
		echo "Compilation of obstacle.c completed successfully\n" || \
		echo "Error compiling obstacle.c\n"

target: $(SRC)/target.c
	$(CC) -I$(INC) -o $(BIN)/target $(SRC)/target.c -lrt -pthread -lbsd && \
		echo "Compilation of target.c completed successfully\n" || \
		echo "Error compiling target.c\n"

# Cleanup target to remove generated binaries and logs
clean:
	rm -rf $(BIN)
