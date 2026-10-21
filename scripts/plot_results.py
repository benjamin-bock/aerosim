"""AeroSim telemetry plotter.

Reads the CSV produced by the simulator and renders a multi-panel flight
report: altitude, velocity, Mach number, dynamic pressure (max Q),
acceleration and propellant remaining.

Usage:
    python3 plot_results.py <input_csv> <output_png>

The CSV columns are documented in src/main.cpp (writeCsvHeader); this
script intentionally does not recompute any physics - it only plots the
quantities the simulator logged, so the plot can never disagree with
the simulation by construction.
"""

import sys

import pandas as pd
import matplotlib.pyplot as plt

# Expected columns, mirroring writeCsvHeader() in src/main.cpp.
COLUMNS = [
    "Time_s",
    "Altitude_m",
    "Velocity_mps",
    "Acceleration_mps2",
    "Mass_kg",
    "Fuel_kg",
    "Mach",
    "Drag_N",
    "Thrust_N",
    "DynamicPressure_Pa",
    "Density_kgm3",
    "DragCoefficient",
    "SpeedOfSound_mps",
    "Phase",
]


def main() -> None:
    if len(sys.argv) < 3:
        print("Usage: python plot_results.py <input_csv> <output_png>")
        sys.exit(1)

    csv_file, output_image = sys.argv[1], sys.argv[2]

    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        sys.exit(1)

    for col in COLUMNS:
        if col not in data.columns:
            print(f"Error: expected column '{col}' is missing from {csv_file}.")
            sys.exit(1)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12), sharex=True)
    (ax_alt, ax_mach), (ax_vel, ax_q), (ax_acc, ax_fuel) = axes

    # Altitude vs time, with the apogee marked.
    apogee_idx = data["Altitude_m"].idxmax()
    ax_alt.plot(data["Time_s"], data["Altitude_m"], color="tab:blue")
    ax_alt.plot(
        data["Time_s"][apogee_idx],
        data["Altitude_m"][apogee_idx],
        "ro",
        label=f"Apogee: {data['Altitude_m'][apogee_idx]:.0f} m",
    )
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.grid(True)
    ax_alt.legend()

    # Mach number, with the sonic line M = 1 marked.
    ax_mach.plot(data["Time_s"], data["Mach"], color="tab:purple")
    ax_mach.axhline(1.0, color="black", linewidth=0.8, linestyle="--", label="Mach 1")
    ax_mach.set_ylabel("Mach number [-]")
    ax_mach.grid(True)
    ax_mach.legend()

    # Vertical velocity; zero line marks the apogee crossing.
    ax_vel.plot(data["Time_s"], data["Velocity_mps"], color="tab:red")
    ax_vel.axhline(0, color="black", linewidth=0.5)
    ax_vel.set_ylabel("Velocity [m/s]")
    ax_vel.grid(True)

    # Dynamic pressure: the "max Q" load metric of launch vehicles.
    ax_q.plot(data["Time_s"], data["DynamicPressure_Pa"] / 1000.0, color="tab:orange")
    ax_q.set_ylabel("Dynamic pressure q [kPa]")
    ax_q.grid(True)

    # Vertical acceleration, in g for readability.
    ax_acc.plot(data["Time_s"], data["Acceleration_mps2"] / 9.80665, color="tab:green")
    ax_acc.set_ylabel("Acceleration [g]")
    ax_acc.grid(True)

    # Remaining propellant mass.
    ax_fuel.plot(data["Time_s"], data["Fuel_kg"], color="tab:brown")
    ax_fuel.set_ylabel("Propellant mass [kg]")
    ax_fuel.set_xlabel("Time [s]")
    ax_fuel.grid(True)

    fig.suptitle("AeroSim Flight Telemetry", fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(output_image)
    print(f"Plot saved as {output_image}")


if __name__ == "__main__":
    main()
