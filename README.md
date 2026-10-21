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

## ✅ Verification

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
├── include/             # Headers (constants, atmosphere, rocket, config)
├── src/                  # Implementation + physics verification suite
├── scripts/             # Python plotting (no physics recomputed)
├── output/              # Flight data (CSV) and graphs (PNG) — generated
└── build/               # Object files and executables — generated
```
