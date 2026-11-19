import sys
import os
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python plot_design.py <output_dir>")
    sys.exit(1)

outdir = sys.argv[1]
cons_csv = os.path.join(outdir, "constraints.csv")
weights_csv = os.path.join(outdir, "weights.csv")
carpet_csv = os.path.join(outdir, "carpet.csv")

# --- Matching chart: required T/W & P/W vs wing loading, one line per constraint
if os.path.exists(cons_csv):
    cons = pd.read_csv(cons_csv)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    labels = cons['label'].unique()
    colors = plt.cm.tab10(range(len(labels)))
    cmap = dict(zip(labels, colors))
    for lbl in labels:
        sub = cons[(cons['label'] == lbl) & (cons['kind'] != 0)]
        if sub.empty:
            continue
        ax1.plot(sub['ws'], sub['tw'], color=cmap[lbl], label=lbl)
        ax2.plot(sub['ws'], sub['pw'], color=cmap[lbl], label=lbl)
    stall = cons[cons['kind'] == 0]
    if not stall.empty:
        for ax in (ax1, ax2):
            ax.axvline(float(stall['ws'].iloc[0]), color='red', linestyle='--', label='stall limit')
    for ax, ylab in ((ax1, 'Required T/W'), (ax2, 'Required P/W [W/N]')):
        ax.set_xlabel('Wing loading W/S [N/m²]')
        ax.set_ylabel(ylab)
        ax.grid(True)
        ax.legend()
    ax1.set_title('Matching chart (jet convention)')
    ax2.set_title('Matching chart (prop convention)')
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "matching_chart.png"), dpi=150)
    print(f"Saved {outdir}/matching_chart.png")

# --- Weight breakdown pie
if os.path.exists(weights_csv):
    w = pd.read_csv(weights_csv)
    w = w.dropna()
    w = w[w['kg'] > 0]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.pie(w['kg'], labels=w['component'], autopct='%1.1f%%', startangle=90)
    ax.set_title('Weight breakdown')
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "weights.png"), dpi=150)
    print(f"Saved {outdir}/weights.png")

# --- Carpet plot: W0 contours over AR x W/S
if os.path.exists(carpet_csv):
    c = pd.read_csv(carpet_csv)
    conv = c[c['converged'] == 1]
    if not conv.empty:
        fig, ax = plt.subplots(figsize=(10, 7))
        ars = sorted(conv['ar'].unique())
        for ar in ars:
            sub = conv[conv['ar'] == ar].sort_values('ws')
            ax.plot(sub['ws'], sub['w0_kg'], marker='o', label=f'AR={ar}')
        best = conv.loc[conv['w0_kg'].idxmin()]
        ax.plot(best['ws'], best['w0_kg'], 'r*', markersize=20,
                label=f"optimum (AR={best['ar']:.0f}, W/S={best['ws']:.0f})")
        ax.set_xlabel('Wing loading W/S [N/m²]')
        ax.set_ylabel('Takeoff weight W0 [kg]')
        ax.set_title('Trade study: W0 vs W/S for several aspect ratios')
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, "carpet.png"), dpi=150)
        print(f"Saved {outdir}/carpet.png")
