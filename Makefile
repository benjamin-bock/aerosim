# Directory paths
SRC_DIR = src
INC_DIR = include
OBJ_DIR = build
OUT_DIR = output
SCRIPT_DIR = scripts

# Executable target
TARGET = $(OBJ_DIR)/aerosim

# Compiler and flags
CXX = g++
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O3

# Files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = scripts/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png

# Arguments for the simulation
ARGS =

# --- RULES ---

all: $(TARGET)

# Compile the executable
$(TARGET): $(OBJS)
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(OBJS) -o $(TARGET)
	@echo "Compilation finished: $(TARGET)"

# Compile object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to run everything at once
run: $(TARGET)
	@echo "Launching simulation..."
	./$(TARGET) ${ARGS}
	@echo "Generating plot..."
	$(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG)

# Cleanup
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Directories build/ and output/ cleaned."

.PHONY: all clean run