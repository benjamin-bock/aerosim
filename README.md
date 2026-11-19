# 🚀 AeroSim: Vertical Rocket Flight Simulator

![Language](https://img.shields.io/badge/language-C%2B%2B17-blue.svg)
![Status](https://img.shields.io/badge/status-Alpha-orange.svg)

**AeroSim** simulates the complete vertical flight of a rocket —
powered ascent, ballistic coast, apogee, drag descent, landing — in
C++17, with an automated analysis pipeline in Python.

**This tool is designed not to be a black box.** Every equation,
constant, and numerical choice is documented with its physical
derivation and bibliographic source in
[docs/PHYSICS.md](docs/PHYSICS.md), and each physics component is
verified against published reference tables or closed-form analytical
solutions by a built-in test suite (`make test`).

## 📐 Physics implemented

| Model | Description | Reference |
| :--- | :--- | :--- |
| Atmosphere | Layered **US Standard Atmosphere 1976** (8 layers, 0–86 km): piecewise-linear temperature, exact hydrostatic pressure integration | [COESA76] Table 4 |
| Speed of sound | Computed from local temperature: `a = sqrt(γ·R·T)` | [COESA76] §1.3 |
| Gravity | Standard `g0 = 9.80665 m/s²` (the value the 1976 Standard is defined around) | [COESA76] §1.2 |
| Drag | `D = ½ρv²·Cd(M)·A`, with a three-regime Mach-dependent Cd (subsonic constant, transonic ramp to 2×Cd0, supersonic 1/√M decay) | [Sutton] Ch. 3, [Anderson] |
| Propellant | `ṁ = sfc·T`, exactly consistent with the ideal thrust equation `T = ṁ·vₑ` (Tsiolkovsky-consistent) | [Sutton] eq. 2-14 |
| Integrator | **Semi-implicit (symplectic) Euler** — energy-bounded, unlike explicit Euler | [Hairer] §II.16 |

Full derivations, justifications, and limitations: [docs/PHYSICS.md](docs/PHYSICS.md).

## 🛠 Installation

```bash
# C++ side: only g++ (C++17) and make
# Python side (optional, for plots): pandas + matplotlib
python3 -m venv .venv && source .venv/bin/activate
pip install pandas matplotlib
```

## 🚀 Quick start

```bash
make run           # build + simulate with defaults + plot
make test          # build + run the physics verification suite
```

Custom mission:

```bash
make run ARGS="-m 80 -t 6000 -f 120 -fc 0.0004 -cd 0.35"
```

| Flag | Description | Unit | Constraint |
| :--- | :--- | :--- | :--- |
| `-m` | Dry (structural) mass | kg | > 0 |
| `-t` | Engine thrust | N | ≥ 0 |
| `-f` | Initial propellant mass | kg | ≥ 0 |
| `-dt` | Integration time step | s | > 0 |
| `-g` | Gravitational acceleration | m/s² | > 0 |
| `-fc` | Specific fuel consumption | kg/(N·s) | ≥ 0 |
| `-a` | Reference (frontal) area | m² | > 0 |
| `-cd` | Subsonic drag coefficient | – | ≥ 0 |
| `-o` | Output CSV path | path | – |
| `-h` | Help | | |

The simulator prints a mission summary (apogee, max Mach, max Q, max
acceleration, burnout and touchdown times) and writes full telemetry
(time, altitude, velocity, acceleration, mass, fuel, Mach, drag, thrust,
dynamic pressure, density, Cd, speed of sound, flight phase) to CSV.

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

## ✅ Verification

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

The test suite (`src/test_physics.cpp`) checks the physics against
**independent references**, not against the implementation itself:

- atmosphere reproduced from the published 1976 tables at 0–71 km;
- closed-form vacuum ballistics (burn + coast parabola, apogee `v²/2g`);
- analytical burnout time `m_fuel/(sfc·T)`;
- continuity of the piecewise Cd(M) curve at its breakpoints.

```bash
make test
```

## 📁 Project structure

```text
├── docs/PHYSICS.md      # Complete physics reference: every equation sourced
├── include/             # Headers (constants, atmosphere, rocket, PlaneDesign)
├── src/                  # Rocket and aircraft implementations
├── scripts/              # Python plotting tools
├── output/               # Flight data, design CSVs and graphs — generated
└── build/                # Object files and executables — generated
```
