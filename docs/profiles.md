# Reference profiles for hybrid STOL design studies (AIAA DBF)

These candidate sections are common starting points for STOL/low-Re design.
Generate polars with, e.g.:

    make doc
    ./build/aerodoc polar naca4412 -re 1e6 -summary

## High-lift candidates
- NACA 4412 / 4415 / 4418: classic cambered sections, robust, easy to analyze.
- NACA 23012: popular GA section, good Cl_max but sharp stall.
- NACA 6409 / 6412: high camber, high lift at low Re, watch nose-down pitching.

## Low-Re / UAV oriented
- SG6043, E387, SD7037, FX63-137: high-lift low-Re sections (fetch the .dat
  from the UIUC airfoil database and use `polar <file.dat>`).

## Slotted/STOL-oriented
- S1223 (Selig): very high lift, high drag; a frequent STOL contestant.
- USA-35B, Clark-Y: classic flat-bottomed sections, forgiving at low Re.

## Notes for the hybrid STOL mission
- Takeoff requirement drives Cl_max (with devices) and T/W.
- The tool reports xtr_top/xtr_bot so you can see transition movement with
  alpha, and Cd splitting (friction vs pressure vs separation penalty).
- Use `sweep` to explore wing loading and static thrust effects on ground run.
