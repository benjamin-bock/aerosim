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
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O3 -MMD -MP

# Files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
PD_TARGET = $(OBJ_DIR)/planedesign
PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = scripts/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png

# Arguments for the simulation
ARGS =

# --- RULES ---

all: $(TARGET)

# Compile the executable
ROCKET_OBJS = $(OBJ_DIR)/main.o $(OBJ_DIR)/environment.o $(OBJ_DIR)/rocket.o
PLANE_OBJS = $(OBJ_DIR)/naca.o $(OBJ_DIR)/atmosphere.o $(OBJ_DIR)/geometry.o \
             $(OBJ_DIR)/aero.o $(OBJ_DIR)/weights.o $(OBJ_DIR)/constraints.o \
             $(OBJ_DIR)/engines.o $(OBJ_DIR)/performance.o $(OBJ_DIR)/synthesis.o \
             $(OBJ_DIR)/trade.o $(OBJ_DIR)/design_main.o
TEST_OBJS = $(PLANE_OBJS:$(OBJ_DIR)/design_main.o=$(OBJ_DIR)/tests.o)

$(TARGET): $(ROCKET_OBJS)
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(ROCKET_OBJS) -o $(TARGET)
	@echo "Compilation finished: $(TARGET)"

# Compile object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Header dependencies
DEPS = $(OBJS:.o=.d)
-include $(DEPS)

%.d: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	@$(CXX) $(CXXFLAGS) -MM -MT $(OBJ_DIR)/$*.o $< > $@

# Rule to run everything at once
run: $(TARGET)
	@echo "Launching simulation..."
	./$(TARGET) ${ARGS}
	@echo "Generating plot..."
	$(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG)

# Aircraft conceptual design tool
design: $(PD_TARGET)
	@mkdir -p $(OUT_DIR)
	@echo "Running aircraft conceptual design..."
	./$(PD_TARGET) ${ARGS}
	@if [ -x "$(PYTHON)" ]; then $(PYTHON) scripts/plot_design.py $(OUT_DIR); else echo "Skipping plots: $(PYTHON) not found"; fi

$(PD_TARGET): $(PLANE_OBJS)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(PLANE_OBJS) -o $(PD_TARGET)
	@echo "Built: $(PD_TARGET)"

# Unit tests
test: $(TEST_OBJS)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(TEST_OBJS) -o $(OBJ_DIR)/tests -lm
	./$(OBJ_DIR)/tests

# Cleanup
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Directories build/ and output/ cleaned."

.PHONY: all clean run design