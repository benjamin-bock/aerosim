# 🚀 AeroSim: Rocket Trajectory Simulator

![Language](https://img.shields.io/badge/language-C%2B%2B17-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-Alpha-orange.svg)

**AeroSim v0.1** is a ballistic rocket flight simulator written in **C++17**, featuring an automated analysis pipeline in **Python**. It models atmospheric flight physics, including air density variation and a dynamic drag coefficient ($C_d$) based on Mach number (transonic/supersonic).

## 📊 Features
- **Realistic Physics**: Semi-implicit Euler integration, exponential atmosphere model.
- **Drag Model**: $C_d$ calculation based on regime (subsonic, transonic peak, supersonic).
- **Automated Pipeline**: Compilation, simulation, and graph generation in a single command.
- **Flexible**: Fully configurable via command-line arguments.

## 🛠 Installation

### Prerequisites
- `g++` compiler (supporting C++17)
- `make`
- `Python 3` with a virtual environment installed in `.venv/`
- Python libraries: `pandas`, `matplotlib`

```bash
# Install Python dependencies
source .venv/bin/activate
pip install pandas matplotlib
```

## 🚀 Quick Start

To compile and run a standard simulation with automatic graph generation:

```bash
make run
```

To test a specific configuration (e.g., a heavy rocket with high thrust):

```bash
make run ARGS="-m 3000 -t 80000 -f 1500 -fc 0.0005"
```

Available arguments:

| Argument | Description | Unit |
| :--- | :--- | :--- |
| `-m` | Dry mass | kg |
| `-t` | Engine thrust | N |
| `-f` | Initial fuel mass | kg |
| `-dt` | Time step | s |
| `-g` | Gravity | m/s² |
| `-fc` | Consumption ratio | kg/N/s |
| `-a` | Frontal area | m² |
| `-cd` | Base drag coefficient | - |

## 📁 Project Structure

```text
.
├── src/        # C++ source files
├── include/    # C++ headers
├── scripts/    # Python script for visualization
├── output/     # Flight data (CSV) and graphs (PNG)
└── build/      # Object files and executable
```



---

# \u2708\ufe0f AeroDoc: Airfoil & STOL Design Tool

AeroSim now includes **AeroDoc**, a self-contained C++ airfoil analysis and
hybrid-STOL sizing tool built for AIAA Design/Build/Fly-style studies. No
external dependencies, no XFOIL install needed.

## What it does

### 1. Viscous airfoil analysis (XFOIL-like, from scratch)
- **Panel method**: Hess-Smith linear-vortex/source formulation with an
  explicit Kutta condition, cosine-clustered panels at the leading edge.
- **Viscous coupling**: boundary-layer displacement effect via a
  transpiration (source strength) iteration, under-relaxed for stability.
- **Laminar BL**: Thwaites' integral method (exact integral form).
- **Transition**: Michel's criterion Re_\u03b8 = 2.9 Re_x^0.4, with laminar
  separation bubble capture (transition by bubble).
- **Turbulent BL**: Head's entrainment method with Cebeci & Bradshaw
  closures: H1(H), F(H1)=0.0306(H1-3)^-0.6169, Ludwieg-Tillmann skin
  friction.
- **Profile drag**: Squire-Young extrapolation from the trailing-edge
  momentum thickness, plus a separation drag penalty when the turbulent
  boundary layer separates.
- **Stall behavior**: lift attenuation and drag rise as separation moves
  forward; polar sweeps stop naturally near Cl_max.
- **Compressibility**: Prandtl-Glauert/Karman-Tsien style correction at
  moderate subsonic Mach.

### 2. Aircraft-level STOL sizing (hybrid-electric)
- Stall speed, ground run (hybrid-boosted), total takeoff over 50 ft,
  landing distance, L/D max, rate of climb, range/endurance with a simple
  hybrid energy model (fuel + battery), wing/power/disk loading.
- Parametric sweeps: takeoff distance vs wing loading or static thrust.
- Batch mode: compare design candidates from a CSV spec file.

## Quick start

```bash
make doc          # build the design tool
./build/aerodoc polar naca4412 -re 3e6 -summary      # viscous polar
./build/aerodoc polar naca2412 -re 1e6 -csv polar.csv # export CSV
./build/aerodoc geom naca4415                          # geometry info
./build/aerodoc stol -mtow 1500 -S 16 -b 12 -T 4200    # STOL sizing
./build/aerodoc sweep -what wing -lo 60 -hi 200 -n 20  # parametric sweep
./build/aerodoc stol-batch examples/stol_candidates.csv
make test         # regression tests
```

Profiles: any NACA 4/5/6-digit designation, or a Selig-format `.dat` file
(UIUC airfoil database) passed by path.

## Method summary (for your design report)

| Quantity | Method |
|---|---|
| Inviscid edge velocity | Hess-Smith vortex+source panel method |
| Laminar BL | Thwaites |
| Transition | Michel / laminar separation bubble |
| Turbulent BL | Head entrainment (Cebeci-Bradshaw closures) |
| Skin friction | Ludwieg-Tillmann |
| Profile drag | Squire-Young + separation penalty |
| Takeoff ground run | Energy approach with hybrid-boosted acceleration |
| Landing | 1.15 V_stall approach + braking decel |
| Range | Breguet-style fuel + electric segment |

## Files added

```text
src/xf.cpp            # panel + BL viscous solver engine
src/boundary_layer.cpp# Thwaites / Michel / Head BL marcher
src/profile.cpp       # NACA generator + Selig loader
src/aircraft.cpp      # STOL / hybrid sizing module
src/aerodoc.cpp       # CLI
include/*.h           # headers
docs/profiles.md      # profile selection guide
examples/             # batch design candidates
```
