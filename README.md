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

## ✈️ Aircraft Conceptual Design (PlaneDesign)

A Raymer-based conceptual design tool: give it a few guesses (payload, range,
stall speed, takeoff distance, climb rate, NACA airfoil...), it sizes
everything else iteratively.

```bash
make design ARGS="-pay 180 -range 1000 -ar 8 -af 2412"
```

It performs:
- **Weight sizing** (Raymer ch. 3): fixed-point iteration on W0 with the
  empty-weight regression We/W0 = A·W0^c (Table 3.1) and mission fuel fractions
  via Breguet range/endurance equations.
- **NACA airfoils**: 4-digit, 5-digit and 6-series geometries with thin-airfoil
  zero-lift angle and empirical CLmax.
- **Drag buildup** (Raymer ch. 12): equivalent skin-friction method, Oswald
  efficiency from AR (eq. 12.48).
- **Constraint analysis** (Raymer ch. 5/18): required thrust/weight (or power/
  weight for prop aircraft) vs wing loading for stall, takeoff over obstacle,
  climb, cruise and sustained-turn requirements; exports a matching-style CSV.
- **Lift & thrust sizing**: wing area from stall speed, thrust/power from the
  governing performance constraint.

Outputs a full design report to the console, `output/design_summary.csv` and
`output/constraint_analysis.csv` (plots via `scripts/plot_design.py`).

| Argument | Description | Unit |
| :--- | :--- | :--- |
| `-pay` | Payload | kg |
| `-crew` | Crew weight | kg |
| `-range` | Cruise range | km |
| `-vc` | Cruise speed | m/s |
| `-alt` | Cruise altitude | m |
| `-end` | Loiter endurance | h |
| `-vs` | Max stall speed | m/s |
| `-to` | Takeoff distance (over 50 ft obstacle) | m |
| `-roc` | Rate of climb at `-vcl` | m/s |
| `-vmax` | Max level speed | m/s |
| `-n` | Sustained-turn load factor (0 = skip) | - |
| `-ar` | Wing aspect ratio | - |
| `-af` | NACA airfoil (e.g. `2412`, `23012`, `63-209`, `64(2)215`) | - |
| `-jet` | Jet propulsion (default: propeller) | - |
| `-flap` | Takeoff flap CLmax increment | - |

## 📁 Project Structure

```text
.
├── src/        # C++ source files
├── include/    # C++ headers
├── scripts/    # Python scripts for visualization
├── output/     # Flight data (CSV) and graphs (PNG)
└── build/      # Object files and executable
```


