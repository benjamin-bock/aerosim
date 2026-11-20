# AeroSim Makefile
# Targets:
#   make        - build the simulator and AeroDoc
#   make run    - build + simulate with ARGS + plot the telemetry
#   make test   - build and run the verification suites
#   make clean  - remove build artifacts and outputs

SRC_DIR = src
INC_DIR = include
OBJ_DIR = build
OUT_DIR = output
SCRIPT_DIR = scripts

TARGET = $(OBJ_DIR)/aerosim
AERODOC = $(OBJ_DIR)/aerodoc
AERODOC_SRC = src/common.cpp src/profile.cpp src/boundary_layer.cpp src/xf.cpp src/aircraft.cpp src/aerodoc.cpp
TEST_TARGET = $(OBJ_DIR)/test_physics
TEST_SRC = $(SRC_DIR)/test_physics.cpp

CXX = g++
CXXFLAGS = -I$(INC_DIR) -Wall -Wextra -std=c++17 -O3 -MMD -MP

SRCS = $(filter-out $(SRC_DIR)/aerodoc.cpp $(SRC_DIR)/test_physics.cpp $(SRC_DIR)/tests.cpp $(SRC_DIR)/design_main.cpp $(SRC_DIR)/calibration.cpp, $(wildcard $(SRC_DIR)/*.cpp))
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
ROCKET_OBJS = $(OBJ_DIR)/main.o $(OBJ_DIR)/config.o $(OBJ_DIR)/atmosphere.o $(OBJ_DIR)/rocket.o
PLANE_OBJS = $(OBJ_DIR)/naca.o $(OBJ_DIR)/atmosphere.o $(OBJ_DIR)/geometry.o \
             $(OBJ_DIR)/aero.o $(OBJ_DIR)/weights.o $(OBJ_DIR)/constraints.o \
             $(OBJ_DIR)/engines.o $(OBJ_DIR)/performance.o $(OBJ_DIR)/synthesis.o \
             $(OBJ_DIR)/trade.o $(OBJ_DIR)/propeller.o $(OBJ_DIR)/stability.o
TEST_OBJS = $(PLANE_OBJS) $(OBJ_DIR)/tests.o
PD_TARGET = $(OBJ_DIR)/planedesign
PYTHON = ./.venv/bin/python3
PLOT_SCRIPT = $(SCRIPT_DIR)/plot_results.py
CSV = $(OUT_DIR)/flight_data.csv
PNG = $(OUT_DIR)/flight_data.png
ARGS =

all: $(TARGET) $(AERODOC)

$(TARGET): $(ROCKET_OBJS)
	@mkdir -p $(OBJ_DIR) $(OUT_DIR)
	$(CXX) $(ROCKET_OBJS) -o $(TARGET)
	@echo "Compilation finished: $(TARGET)"

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(AERODOC): $(AERODOC_SRC)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(AERODOC_SRC) -o $(AERODOC)
	@echo "Built: $(AERODOC)"

doc: $(AERODOC)

DEPS = $(OBJS:.o=.d)
-include $(DEPS)

run: $(TARGET)
	@mkdir -p $(OUT_DIR)
	@echo "Running simulation..."
	./$(TARGET) $(ARGS)
	@test -x "$(PYTHON)" || echo "Note: .venv/bin/python3 not found; skipping plot. Create it or run: python3 $(PLOT_SCRIPT) $(CSV) $(PNG)"
	@if test -x "$(PYTHON)"; then $(PYTHON) $(PLOT_SCRIPT) $(CSV) $(PNG); fi

design: $(PD_TARGET)
	@mkdir -p $(OUT_DIR)
	@echo "Running aircraft conceptual design..."
	./$(PD_TARGET) $(ARGS)
	@if [ -x "$(PYTHON)" ]; then $(PYTHON) scripts/plot_design.py $(OUT_DIR); else echo "Skipping plots: $(PYTHON) not found"; fi

$(PD_TARGET): $(PLANE_OBJS) $(OBJ_DIR)/design_main.o
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(PLANE_OBJS) $(OBJ_DIR)/design_main.o -o $(PD_TARGET)
	@echo "Built: $(PD_TARGET)"

calibrate: $(OBJ_DIR)/calibration.o $(PLANE_OBJS)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(PLANE_OBJS) $(OBJ_DIR)/calibration.o -o $(OBJ_DIR)/calibrate -lm
	./$(OBJ_DIR)/calibrate

test: $(AERODOC) $(TARGET) $(TEST_TARGET) $(TEST_OBJS)
	@echo "Running AeroDoc regression tests..."
	@$(AERODOC) polar naca2412 -re 3e6 -a0 -2 -a1 4 -da 2 -summary | grep -q "Cl_max" || (echo "FAIL: polar 2412"; exit 1)
	@$(AERODOC) polar naca4412 -re 1e6 -a0 0 -a1 8 -da 2 -summary | grep -q "Cl_slope" || (echo "FAIL: polar 4412"; exit 1)
	@$(AERODOC) geom naca2412 | grep -q "0.1200" || (echo "FAIL: geom"; exit 1)
	@$(AERODOC) stol -mtow 1500 -S 16 -b 12 | grep -q "STOL Analysis" || (echo "FAIL: stol"; exit 1)
	@$(AERODOC) sweep -what wing -lo 60 -hi 150 -n 10 | grep -q "Takeoff" || (echo "FAIL: sweep"; exit 1)
	@echo "Running rocket and aircraft unit tests..."
	./$(TEST_TARGET)
	$(CXX) $(CXXFLAGS) $(TEST_OBJS) -o $(OBJ_DIR)/tests -lm
	./$(OBJ_DIR)/tests
	@echo "All tests passed."

$(TEST_TARGET): $(TEST_SRC) $(ROCKET_OBJS)
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(filter-out $(OBJ_DIR)/main.o,$(ROCKET_OBJS)) -o $@
	@echo "Built test suite: $@"

clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)
	@echo "Cleaned build/ and output/."

.PHONY: all clean run doc design calibrate test
