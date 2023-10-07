# Call this with: [make clean], [make build] (same as [make]), [make gomain]

# Notes:
# ... $< stands for: first from dependencies
# ... $^ stands for: all from dependency list
# ... $@ stands for: target element

SRC_DIR := ./src/
INC_DIR := ./inc/
STARTUP_DIR := ./startup/
LINK_DIR := ./link/
OUT_DIR := ./out/
OBJ_DIR := ./out/obj/

LINKER := linker_script.ld
ELF := out.elf
MAP := out.map
STARTUP := startup.s

CFLAGS := -c -O0 -g3 -mcpu=cortex-m4 -mthumb
LFLAGS := --print-memory-usage

# Get files with path
LINKER_FILE := $(addprefix $(LINK_DIR),$(LINKER))
ELF_FILE := $(addprefix $(OUT_DIR),$(ELF))
MAP_FILE := $(addprefix $(OUT_DIR),$(MAP))
STARTUP_FILE := $(addprefix $(STARTUP_DIR),$(STARTUP))

# Get source files list
SOURCES := $(wildcard $(addprefix $(SRC_DIR),*.c))

# Get headers list
HEADERS := $(wildcard $(addprefix $(INC_DIR),*.h))

# Get startup files list
STARTUPS := $(wildcard $(addprefix $(STARTUP_DIR),*.s))

# Replace .c with .o in source files list, then replace source dir with obj dir to obtain object files list
TEMP := $(subst .c,.o,$(SOURCES))
OBJECTS := $(subst $(SRC_DIR),$(OBJ_DIR),$(TEMP))

# Replace .s with .o in startup files list, then replace startup dir with obj dir to obtain startup-object files list
TEMP_S := $(subst .s,.o,$(STARTUPS))
OBJECTS_S := $(subst $(STARTUP_DIR),$(OBJ_DIR),$(TEMP_S))

# Link all .o files from OBJECTS list to an executable program
$(ELF_FILE): $(OBJECTS) $(OBJECTS_S)
	@echo "Build elf"
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-ld -T $(LINKER_FILE) -Map=$(MAP_FILE) $(LFLAGS) $^ -o $@

# Compile each .c file from SRC_DIR to a corresponding .o file inside OBJ_DIR
$(OBJ_DIR)%.o: $(SRC_DIR)%.c $(HEADERS)
	@echo "Compile sources"
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc -o $@ $(CFLAGS) $< -I$(INC_DIR)
	
# Compile each .s file from STARTUP_DIR to a corresponding .o file inside OBJ_DIR
$(OBJ_DIR)%.o: $(STARTUP_DIR)%.s
	@echo "Compile startup files"
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc -o $@ $(CFLAGS) $<

# Don't make target files from keywords clean, build
.PHONY: clean run build

# Clean up: call with make clean
clean:
	@echo "Clean"
	rm -f $(ELF_FILE)
	rm -f $(MAP_FILE)
	rm -f $(OBJ_DIR)*.o
	
# Open st-util in another terminal, open gdb in the current terminal and load elf on the target, set breakpoint in main and go
gomain:
	gnome-terminal -- sh -c "bash -c \"st-util; exec bash\""
	sleep 3s
	../arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb $(ELF_FILE) -quiet -ex 'target extended localhost:4242' -ex 'load' -ex 'b main' -ex 'continue'

# Build executable: call with "make build" or just plain "make"
build: $(ELF_FILE)
.DEFAULT_GOAL := $(ELF_FILE)
