# AeroSim Makefile
# Targets:
#   make        - build the simulator
#   make run    - build + simulate with ARGS + plot the telemetry
#   make test   - build and run the physics verification suite
#   make clean  - remove build artifacts and outputs

SRC_DIR = src
INC_DIR = include
OBJ_DIR = build
OUT_DIR = output
SCRIPT_DIR = scripts

# The test executable is built from test_physics.cpp only, so the main
# target must not try to link it (it has its own main()).
MAIN_SRCS = $(filter-out $(SRC_DIR)/test_physics.cpp,$(wildcard $(SRC_DIR)/*.cpp))
MAIN_OBJS = $(MAIN_SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
TEST_SRC = $(SRC_DIR)/test_physics.cpp

TARGET = $(OBJ_DIR)/aerosim
TEST_TARGET = $(OBJ_DIR)/test_physics

CXX = g++
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O2

PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = $(SCRIPT_DIR)/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png

# Extra arguments passed to the simulator, e.g.:
#   make run ARGS="-m 80 -t 6000"
ARGS =

all: $(TARGET)

$(TARGET): $(MAIN_OBJS)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(MAIN_OBJS) -o $@
	@echo "Built simulator: $@"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Physics verification suite: atmosphere tables, closed-form ballistics,
# burnout time, Cd(M) continuity. See src/test_physics.cpp.
test: $(TARGET) $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): $(TEST_SRC) $(TARGET)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I$(INC_DIR) $(TEST_SRC) $(filter-out $(OBJ_DIR)/main.o,$(MAIN_OBJS)) -o $@
	@echo "Built test suite: $@"

run: $(TARGET)
	@mkdir -p $(OUT_DIR)
	@echo "Running simulation..."
	./$(TARGET) $(ARGS)
	@test -x "$(PYTHON)" || echo "Note: .venv/bin/python3 not found; skipping plot. Create it or run: python3 $(PLOT_SCRIPT) $(CSV) $(PNG)"
	@if test -x "$(PYTHON)"; then $(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG); fi

clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Cleaned build/ and output/."

.PHONY: all run test clean
