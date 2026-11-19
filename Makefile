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
MAIN_SRCS = $(SRC_DIR)/main.cpp $(SRC_DIR)/config.cpp $(SRC_DIR)/atmosphere.cpp $(SRC_DIR)/rocket.cpp
MAIN_OBJS = $(MAIN_SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
TEST_SRC = $(SRC_DIR)/test_physics.cpp

TARGET = $(OBJ_DIR)/aerosim
TEST_TARGET = $(OBJ_DIR)/test_physics

CXX = g++
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O3 -MMD -MP

# Files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
PD_TARGET = $(OBJ_DIR)/planedesign
PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = $(SCRIPT_DIR)/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png

# Extra arguments passed to the simulator, e.g.:
#   make run ARGS="-m 80 -t 6000"
ARGS =

all: $(TARGET)

# Compile the executable
ROCKET_OBJS = $(OBJ_DIR)/main.o $(OBJ_DIR)/config.o $(OBJ_DIR)/atmosphere.o $(OBJ_DIR)/rocket.o
PLANE_OBJS = $(OBJ_DIR)/naca.o $(OBJ_DIR)/atmosphere.o $(OBJ_DIR)/geometry.o \
             $(OBJ_DIR)/aero.o $(OBJ_DIR)/weights.o $(OBJ_DIR)/constraints.o \
             $(OBJ_DIR)/engines.o $(OBJ_DIR)/performance.o $(OBJ_DIR)/synthesis.o \
             $(OBJ_DIR)/trade.o $(OBJ_DIR)/design_main.o \
             $(OBJ_DIR)/propeller.o $(OBJ_DIR)/stability.o
TEST_OBJS = $(PLANE_OBJS:$(OBJ_DIR)/design_main.o=$(OBJ_DIR)/tests.o)

$(TARGET): $(ROCKET_OBJS)
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(ROCKET_OBJS) -o $(TARGET)
	@echo "Compilation finished: $(TARGET)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Header dependencies
DEPS = $(OBJS:.o=.d)
-include $(DEPS)

%.d: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	@$(CXX) $(CXXFLAGS) -MM -MT $(OBJ_DIR)/$*.o $< > $@

# Run the rocket simulator
run: $(TARGET)
	@mkdir -p $(OUT_DIR)
	@echo "Running simulation..."
	./$(TARGET) $(ARGS)
	@test -x "$(PYTHON)" || echo "Note: .venv/bin/python3 not found; skipping plot. Create it or run: python3 $(PLOT_SCRIPT) $(CSV) $(PNG)"
	@if test -x "$(PYTHON)"; then $(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG); fi

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

# Calibration against Cessna 172
calibrate: $(OBJ_DIR)/calibration.o $(PLANE_OBJS:$(OBJ_DIR)/design_main.o=)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(filter-out $(OBJ_DIR)/design_main.o,$(PLANE_OBJS)) $(OBJ_DIR)/calibration.o -o $(OBJ_DIR)/calibrate -lm
	./$(OBJ_DIR)/calibrate

# Cleanup
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Cleaned build/ and output/."

# Rocket and aircraft unit tests
test: $(TARGET) $(TEST_TARGET) $(TEST_OBJS)
	./$(TEST_TARGET)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(TEST_OBJS) -o $(OBJ_DIR)/tests -lm
	./$(OBJ_DIR)/tests

$(TEST_TARGET): $(TEST_SRC) $(TARGET)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -I$(INC_DIR) $(TEST_SRC) $(filter-out $(OBJ_DIR)/main.o,$(MAIN_OBJS)) -o $@
	@echo "Built test suite: $@"

.PHONY: all clean run design test calibrate
