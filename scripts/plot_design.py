import pandas as pd
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 3:
    print("Usage: python plot_design.py <design_summary_csv> <constraint_csv>")
    sys.exit(1)

summary_csv, constraint_csv = sys.argv[1], sys.argv[2]

try:
    summary = pd.read_csv(summary_csv)
except FileNotFoundError:
    print(f"Error: {summary_csv} not found.")
    sys.exit(1)

try:
    cons = pd.read_csv(constraint_csv)
except FileNotFoundError:
    print(f"Error: {constraint_csv} not found.")
    sys.exit(1)

vals = dict(zip(summary['parameter'], summary['value']))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Weight breakdown pie
w0 = float(vals['W0'])
we = float(vals['We'])
wf = float(vals['Wf'])
labels = ['Empty', 'Fuel', 'Payload+Crew']
payload = w0 - we - wf
ax1.pie([we, wf, max(payload, 0)], labels=labels, autopct='%1.1f%%', startangle=90)
ax1.set_title(f"Weight breakdown - W0 = {w0:.0f} kg")

# Constraint analysis: required P/W vs wing loading
ok = cons[cons['constraint'] == 'ok']
ax2.plot(ok['ws_N_m2'], ok['pw_required_W_kg'], color='blue', label='Required P/W')
ax2.axhline(float(vals['power_required']) / w0, color='red', linestyle='--',
           label='Design P/W')
ax2.axvline(w0 * 9.81 / float(vals['wing_area']), color='green', linestyle=':',
            label='Design W/S')
ax2.set_xlabel('Wing loading W/S [N/m^2]')
ax2.set_ylabel('Required P/W [W/kg]')
ax2.grid(True)
ax2.legend()
ax2.set_title('Constraint analysis (prop-driven)')

plt.tight_layout()
out = summary_csv.replace('.csv', '.png')
plt.savefig(out, dpi=150)
print(f"Design plot saved to {out}")
