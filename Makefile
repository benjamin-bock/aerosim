# Directory paths
SRC_DIR = src
INC_DIR = include
OBJ_DIR = build
OUT_DIR = output
SCRIPT_DIR = scripts

# Executable targets
TARGET = $(OBJ_DIR)/aerosim
AERODOC = $(OBJ_DIR)/aerodoc
AERODOC_SRC = src/common.cpp src/profile.cpp src/boundary_layer.cpp src/xf.cpp src/aircraft.cpp src/aerodoc.cpp

# Compiler and flags
CXX = g++
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O3

# Files
SRCS = $(filter-out $(SRC_DIR)/aerodoc.cpp, $(wildcard $(SRC_DIR)/*.cpp))
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = scripts/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png

# Arguments for the simulation
ARGS =

# --- RULES ---

all: $(TARGET) $(AERODOC)

# Compile the rocket simulator
$(TARGET): $(OBJS)
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(OBJS) -o $(TARGET)
	@echo "Compilation finished: $(TARGET)"

# Compile object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile the airfoil / STOL design tool
$(AERODOC): $(AERODOC_SRC)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(AERODOC_SRC) -o $(AERODOC)
	@echo "Built: $(AERODOC)"

doc: $(AERODOC)

# Rule to run everything at once
run: $(TARGET)
	@echo "Launching simulation..."
	./$(TARGET) ${ARGS}
	@echo "Generating plot..."
	$(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG)

# Regression tests for the design tool
test: $(AERODOC)
	@echo "Running regression tests..."
	@$(AERODOC) polar naca2412 -re 3e6 -a0 -2 -a1 4 -da 2 -summary | grep -q "Cl_max" || (echo "FAIL: polar 2412"; exit 1)
	@$(AERODOC) polar naca4412 -re 1e6 -a0 0 -a1 8 -da 2 -summary | grep -q "Cl_slope" || (echo "FAIL: polar 4412"; exit 1)
	@$(AERODOC) geom naca2412 | grep -q "0.1200" || (echo "FAIL: geom"; exit 1)
	@$(AERODOC) stol -mtow 1500 -S 16 -b 12 | grep -q "STOL Analysis" || (echo "FAIL: stol"; exit 1)
	@$(AERODOC) sweep -what wing -lo 60 -hi 150 -n 10 | grep -q "Takeoff" || (echo "FAIL: sweep"; exit 1)
	@echo "All tests passed."

# Cleanup
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Directories build/ and output/ cleaned."

.PHONY: all clean run doc test
