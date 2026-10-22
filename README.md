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

A full Raymer-based conceptual design synthesis: give it a few guesses
(payload, range, stall speed, takeoff distance, climb rate, NACA airfoil...),
it iterates everything else to convergence in one damped fixed-point loop:

```
W0 → geometry → aerodynamics → constraint sizing → engine selection
   → component weights → Breguet fuel → W0 (until closed)
```

```bash
make design ARGS="-pay 180 -range 800 -ar 8 -af 2412"   # single design
make design ARGS="... -carpet"                           # + AR×W/S trade study
make test                                               # 53 unit tests
make calibrate                                          # validation vs Cessna 172
```

**Validation**: the tool reproduces a Cessna 172 Skyhawk within:
W0 +4.7%, wing area +7.2%, span +3.8%, power +13.5%, stall speed +3.7%,
cruise L/D +16%, range +10% (reserve accounting), ceiling +33%,
and selects the correct engine class (Lycoming).

### Modules (`include/` + `src/`)
| Module | Content |
|---|---|
| `naca` | NACA 4-digit / 5-digit / 6-series airfoils: exact mean line & thickness, thin-airfoil α_L0, empirical CLmax |
| `atmosphere` | Full ISA (T, p, ρ, a, μ) to 20 km |
| `geometry` | Fuselage from cabin sizing, tails from volume coefficients (Raymer ch. 6), wetted areas |
| `aero` | Component drag buildup: Cf/FF/Q (Raymer ch. 12), flaps (plain→double-slot), gear drag, Korn Mdd compressibility, DATCOM 3D lift slope |
| `weights` | Exact Raymer ch. 15.3.3 GA component equations (wing, fuselage, tails, gear, engine installed, fuel system, flight controls, avionics, electrical, furnishings), cross-checked against Table 3.1 |
| `constraints` | One matching-chart line per requirement: takeoff over obstacle, climb, cruise, max speed, sustained turn, ceiling, stall cap |
| `engines` | Off-the-shelf engine database (Rotax, Lycoming, Continental, PT6A...), automatic selection |
| `performance` | Breguet range/endurance, best climb, service ceiling, V-n diagram, integrated takeoff & landing distances |
| `propeller` | eta = f(J) fit to Clark-Y charts, static thrust, diameter trend calibrated on real aircraft (C172/SR22/PT6A) |
| `stability` | Component CG stations, aircraft CG, neutral point with downwash gradient, static margin; wing station solved by bisection for target SM |
| `synthesis` | The damped iteration loop closing weight, aero, power, fuel AND balance |
| `trade` | Carpet study AR × W/S → minimum-W0 optimum (Raymer ch. 18-19) |

### Outputs (`output/`)
- `design_report.md` — full design report (weights table, geometry, aero, propulsion, predicted performance, convergence history)
- `constraints.csv`, `weights.csv`, `carpet.csv` — data for plots
- `matching_chart.png`, `weights.png`, `carpet.png` — via `scripts/plot_design.py` (needs `pandas`/`matplotlib`)

### Key options
`-pay` payload kg · `-range` km · `-vs` stall speed m/s · `-to` takeoff distance m ·
`-roc` climb rate m/s · `-vmax` max speed m/s · `-ar` aspect ratio ·
`-af` NACA airfoil · `-flap` plain|split|single|fowler|double ·
`-jet` jet propulsion · `-retract` retractable gear · `-comp` composite airframe ·
`-seats` · `-engines` · `-carpet` trade study · `-n` sustained turn g · `-ceil` service ceiling m

Example — 2-seater, 1000 km, composite, Fowler flaps, optimum search:
```bash
make design ARGS="-pay 180 -range 1000 -seats 2 -comp -flap fowler -carpet"
```

## 📁 Project Structure

```text
.
├── src/        # C++ source files
├── include/    # C++ headers
├── scripts/    # Python scripts for visualization
├── output/     # Flight data (CSV) and graphs (PNG)
└── build/      # Object files and executable
```


