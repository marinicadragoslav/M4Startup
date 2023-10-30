# Call this with: [make clean], [make build], [make run], [make run_to_main]

# Notes:
# ... this only works for files directly inside the SRC_DIR, INC_DIR, etc, it doesn't find files in subdirectories
# ... $< stands for: first from dependencies
# ... $^ stands for: all from dependency list
# ... $@ stands for: target element

SRC_DIR := ./src/
INC_DIR := ./inc/
STARTUP_DIR := ./startup/
OBJ_DIR := ./out/obj/

LINKER_FILE := ./link/linker_script.ld
ELF_FILE := ./out/out.elf
MAP_FILE := ./out/out.map

GCC := ../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc
LD := ../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-ld 

CFLAGS := -O0 -g3 -mcpu=cortex-m4 -mthumb -Wall -Wextra
LFLAGS := --print-memory-usage

# Get lists of files for each type
C_FILES := $(wildcard $(addprefix $(SRC_DIR),*.c))
H_FILES := $(wildcard $(addprefix $(INC_DIR),*.h))
S_FILES := $(wildcard $(addprefix $(STARTUP_DIR),*.s))

# Replace .c with .o and SRC_DIR with OBJ_DIR to get a list of object files:
O_FILES_FROM_C := $(patsubst %.c,%.o,$(patsubst $(SRC_DIR)%,$(OBJ_DIR)%,$(C_FILES)))

# Replace .s with .o and STARTUP_DIR with OBJ_DIR to get a list of object files:
O_FILES_FROM_S := $(patsubst %.s,%.o,$(patsubst $(STARTUP_DIR)%,$(OBJ_DIR)%,$(S_FILES)))

# Concatenate all object files
O_FILES := $(O_FILES_FROM_C) $(O_FILES_FROM_S)

# Link all .o files to an executable
$(ELF_FILE): $(O_FILES)
	@echo "Building elf"
	$(LD) -T $(LINKER_FILE) -Map=$(MAP_FILE) $(LFLAGS) $^ -o $@

# Compile each .c file to a corresponding .o file
$(O_FILES_FROM_C): $(OBJ_DIR)%.o : $(SRC_DIR)%.c $(H_FILES)
	@echo "Compiling"
	$(GCC) -c -o $@ $(CFLAGS) $< -I$(INC_DIR)

# Compile each .s file to a corresponding .o file
$(O_FILES_FROM_S): $(OBJ_DIR)%.o : $(STARTUP_DIR)%.s
	@echo "Compiling"
	$(GCC) -c -o $@ $(CFLAGS) $<
	
# Don't make target files from keywords clean, build
.PHONY: clean build run run_to_main

# Clean up: call with make clean
clean:
	@echo "Cleaning"
	rm -f $(ELF_FILE)
	rm -f $(MAP_FILE)
	rm -f $(OBJ_DIR)*.o

# Build executable: call with "make build" or just plain "make"
build: $(ELF_FILE)

# Open st-util in another terminal, open gdb in the current terminal and load elf on the target, then go
run:
	gnome-terminal -- sh -c "bash -c \"st-util; exec bash\"" 
	sleep 3s
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb $(ELF_FILE) -quiet -ex 'target extended localhost:4242' -ex 'load' -ex 'continue'

# Open st-util in another terminal, open gdb in the current terminal and load elf on the target, set breakpoint in main and go
run_to_main:
	gnome-terminal -- sh -c "bash -c \"st-util; exec bash\"" 
	sleep 3s
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb $(ELF_FILE) -quiet -ex 'target extended localhost:4242' -ex 'load' -ex 'b main' -ex 'continue'

# Specify what rule is invoked when calling plain "make" (in this case the same as "make build")
.DEFAULT_GOAL := $(ELF_FILE)
