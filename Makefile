# ==========================================================================
# PS2000 Reverse Engineered Library - Makefile
# ==========================================================================
#
# Build system for the reconstructed libps2000 library.
# Supports:
#   - Compilation of all C modules
#   - Generation of init script data from JSON
#   - Shared library output
#
# Usage:
#   make              - Build the shared library
#   make clean        - Remove build artifacts
#   make data         - Regenerate data files from JSON
#   make test         - Build and run tests (if available)
#
# Dependencies:
#   - libusb-1.0-dev
#   - python3 (for data generation tools)
#
# ==========================================================================

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -Werror -O2 -fPIC -Iinclude -std=c11
CFLAGS += -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L
LDFLAGS = -shared -lusb-1.0 -lpthread -lm -ldl

# Directories
SRC_DIR = src
INC_DIR = include
BIN_DIR = build
TOOL_DIR = tools
DATA_DIR = data

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(BIN_DIR)/%.o)
HEADERS = $(wildcard $(INC_DIR)/*.h)

# Target library
TARGET = $(BIN_DIR)/libps2000_re.so

# Data files
INIT_SCRIPT_JSON = $(DATA_DIR)/init_script.json
INIT_SCRIPT_INC = $(SRC_DIR)/init_script_data.inc
CONFIG_JSON = $(DATA_DIR)/config_data.json

# Tools
INIT_TBL_ASM = $(TOOL_DIR)/init_tbl_asm.py
INIT_TBL_DISASM = $(TOOL_DIR)/init_tbl_disasm.py
CFG_ASM = $(TOOL_DIR)/cfg_asm.py
CFG_DISASM = $(TOOL_DIR)/cfg_disasm.py

# ==========================================================================
# PHONY TARGETS
# ==========================================================================

.PHONY: all clean dirs data test help

# Default target
all: dirs $(TARGET)
	@echo ""
	@echo "=========================================="
	@echo "Build complete: $(TARGET)"
	@echo "=========================================="

# Create build directory
dirs:
	@mkdir -p $(BIN_DIR)

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BIN_DIR)
	@rm -f $(INIT_SCRIPT_INC)
	@echo "Clean complete."

# Generate data files from JSON
data: $(INIT_SCRIPT_INC)
	@echo "Data generation complete."

# Run tests (placeholder)
test: all
	@echo "Running tests..."
	@if [ -d tests ]; then \
		$(MAKE) -C tests; \
	else \
		echo "No tests directory found."; \
	fi

# Help
help:
	@echo "PS2000 Reverse Engineered Library - Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build the shared library (default)"
	@echo "  clean   - Remove build artifacts"
	@echo "  data    - Regenerate data files from JSON"
	@echo "  test    - Build and run tests"
	@echo "  help    - Show this help message"
	@echo ""
	@echo "Source files:"
	@for f in $(SOURCES); do echo "  $$f"; done
	@echo ""
	@echo "Dependencies:"
	@echo "  - libusb-1.0-dev"
	@echo "  - python3 (for data generation)"

# ==========================================================================
# COMPILATION RULES
# ==========================================================================

# Link shared library
$(TARGET): $(OBJECTS)
	@echo "Linking $@..."
	@$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "Linked: $@"

# Compile C sources
$(BIN_DIR)/%.o: $(SRC_DIR)/%.c $(HEADERS)
	@echo "Compiling $<..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Special dependency: ps2000_hw.c depends on generated init script
$(BIN_DIR)/ps2000_hw.o: $(SRC_DIR)/ps2000_hw.c $(INIT_SCRIPT_INC) $(HEADERS)
	@echo "Compiling $< (with init script)..."
	@$(CC) $(CFLAGS) -c $< -o $@

# ==========================================================================
# DATA GENERATION RULES
# ==========================================================================

# Generate init script include from JSON
$(INIT_SCRIPT_INC): $(INIT_SCRIPT_JSON) $(INIT_TBL_ASM)
	@echo "Generating init script data..."
	@python3 $(INIT_TBL_ASM) $(INIT_SCRIPT_JSON) --out $@
	@echo "Generated: $@"

# Generate config data include from JSON (if needed)
$(SRC_DIR)/config_data.inc: $(CONFIG_JSON) $(CFG_ASM)
	@echo "Generating config data..."
	@python3 $(CFG_ASM) $(CONFIG_JSON) --out $@
	@echo "Generated: $@"

# ==========================================================================
# UTILITY RULES
# ==========================================================================

# Disassemble init script from binary (for development)
disasm-init: $(INIT_TBL_DISASM)
	@if [ -f $(DATA_DIR)/init_script.bin ]; then \
		python3 $(INIT_TBL_DISASM) $(DATA_DIR)/init_script.bin \
			--stats --out $(INIT_SCRIPT_JSON); \
		echo "Disassembled to $(INIT_SCRIPT_JSON)"; \
	else \
		echo "Error: $(DATA_DIR)/init_script.bin not found"; \
		exit 1; \
	fi

# Show compilation database (for IDE integration)
compile-db:
	@echo "Generating compile_commands.json..."
	@echo "[" > compile_commands.json
	@first=1; \
	for f in $(SOURCES); do \
		if [ $$first -eq 1 ]; then first=0; else echo "," >> compile_commands.json; fi; \
		echo "  {" >> compile_commands.json; \
		echo "    \"directory\": \"$(PWD)\"," >> compile_commands.json; \
		echo "    \"command\": \"$(CC) $(CFLAGS) -c $$f -o $(BIN_DIR)/$$(basename $$f .c).o\"," >> compile_commands.json; \
		echo "    \"file\": \"$$f\"" >> compile_commands.json; \
		echo "  }" >> compile_commands.json; \
	done
	@echo "]" >> compile_commands.json
	@echo "Generated compile_commands.json"

# Static analysis (if cppcheck is available)
analyze:
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --suppress=missingInclude \
			-I$(INC_DIR) $(SRC_DIR)/; \
	else \
		echo "cppcheck not found, skipping analysis"; \
	fi

# Format code (if clang-format is available)
format:
	@if command -v clang-format >/dev/null 2>&1; then \
		clang-format -i $(SRC_DIR)/*.c $(INC_DIR)/*.h; \
		echo "Code formatted"; \
	else \
		echo "clang-format not found, skipping format"; \
	fi

# ==========================================================================
# DEPENDENCY TRACKING
# ==========================================================================

# Include auto-generated dependencies
-include $(OBJECTS:.o=.d)

# Generate dependencies during compilation
$(BIN_DIR)/%.o: CFLAGS += -MMD -MP