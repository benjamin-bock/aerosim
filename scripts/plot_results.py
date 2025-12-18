import pandas as pd
import matplotlib.pyplot as plt
import sys

# Arguments check
# sys.argv[0] is the script name
# sys.argv[1] is the input CSV
# sys.argv[2] is the output image
if len(sys.argv) < 3:
    print("Usage: python plot_results.py <input_csv> <output_png>")
    sys.exit(1)

csv_file = sys.argv[1]
output_image = sys.argv[2]

# Load data
try:
    data = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"Error: {csv_file} not found.")
    sys.exit(1)
# Create a figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Altitude
ax1.plot(data['Time'], data['Altitude'], color='blue', label='Altitude (m)')
ax1.set_ylabel('Altitude [m]')
ax1.grid(True)
ax1.legend()

# Velocity
ax2.plot(data['Time'], data['Velocity'], color='red', label='Velocity (m/s)')
ax2.axhline(0, color='black', linewidth=0.5) # Ligne de l'apogée
ax2.set_ylabel('Velocity [m/s]')
ax2.grid(True)
ax2.legend()

# Fuel
ax3.plot(data['Time'], data['Fuel'], color='green', label='Fuel (kg)')
ax3.set_ylabel('Fuel [kg]')
ax3.set_xlabel('Time [s]')
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.suptitle('AeroSim Flight Telemetry', fontsize=16)
plt.subplots_adjust(top=0.92)

plt.savefig(output_image)
print(f"Plot saved as {output_image}")