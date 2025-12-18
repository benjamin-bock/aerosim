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


