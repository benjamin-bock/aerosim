# AeroSim Physics Reference

This document is the complete, self-contained justification of **every
equation and numerical value** used by AeroSim. Nothing in the code should
be a black box: if a formula or constant appears in the simulation, it is
derived or sourced here, with the reference document and section.

**Scope.** AeroSim simulates the *vertical* (1-D) flight of a thrusting
rocket through the atmosphere: powered ascent, ballistic coast, apogee,
descent under drag, landing. It is a point-mass model: no attitude
dynamics, no wind, no wind shears, no angle of attack.

**Reference list** (short keys used throughout):

| Key | Reference |
|---|---|
| [COESA76] | *U.S. Standard Atmosphere, 1976*, NOAA/NASA/USAF, U.S. GPO, 1976 |
| [Sutton] | Sutton & Biblarz, *Rocket Propulsion Elements*, 9th ed., Wiley, 2017 |
| [Anderson] | Anderson, *Fundamentals of Aerodynamics*, 6th ed., McGraw-Hill |
| [Hairer] | Hairer, Lubich & Wanner, *Geometric Numerical Integration*, 2nd ed., Springer |
| [NASA-EF] | NASA/GSFC *Earth Fact Sheet* (WGS-84 equatorial radius) |

---

## 1. Equations of motion

The rocket is a point mass of time-varying mass `m(t)` moving on the
vertical axis. State: altitude `h [m]`, velocity `v [m/s]` (positive up),
fuel mass `mf [kg]`. Newton's second law along the vertical axis:

```
m * dv/dt = T_active - D - m*g        (1)
dh/dt = v                             (2)
dmf/dt = -sfc * T_active              (3)
```

with:

* `T_active` — thrust [N], equal to the engine thrust `T` while fuel
  remains, 0 after burnout (engine cutoff).
* `D` — aerodynamic drag [N], always opposed to the motion (§4).
* `g` — gravitational acceleration [m/s^2] (§2.4).
* `sfc` — specific fuel consumption, kg of propellant burned per newton
  of thrust per second (§3.2).

Caveat on (1): for a *variable-mass* system the rigorous momentum
balance includes a `\dot m * v_e` term that appears as thrust. Here we
do not double-count it: the engine thrust `T` is specified directly (as
a force), and the mass-flow (3) is only the bookkeeping of how much
propellant that thrust costs. This is the standard "constant-thrust
lumped model" of preliminary rocket trajectory codes [Sutton, Ch. 4].

## 2. Atmosphere model — U.S. Standard Atmosphere 1976

### 2.1 Why 1976 Standard

The US Standard Atmosphere 1976 [COESA76] is the internationally used
reference for flight in the 0–86 km region: it is a piecewise-defined
analytic atmosphere that reproduces mid-latitude mean measurements.
Because the atmosphere is defined analytically, the model introduces
**zero interpolation error**: temperature, pressure, and density are
given by closed-form formulas whose constants are published tables.

The previous AeroSim model used a single exponential law
`rho = rho0 * exp(-h/H)`. That is the *isothermal* exact solution of the
hydrostatic equation (see (7) below) and is a decent first approximation
in a single layer, but it has the wrong scale height near the ground
(it ignores the tropospheric temperature lapse: cold air aloft is
denser at a given pressure), and it is plain wrong in the isothermal
and inversion layers above 11 km. The layered 1976 model removes this
systematic error. Magnitude of the correction: at 11 km the true
density is 0.36392 kg/m^3 while the isothermal fit (fitted at sea level)
predicts ~0.30 kg/m^3 — a ~17% density error on drag at the altitude
where sounding rockets spend most of their supersonic flight.

### 2.2 Geometric vs geopotential altitude

The 1976 Standard Atmosphere is built on the *geopotential* altitude
`H`, in which gravity is treated as constant, `g0 = 9.80665 m/s^2`
[COESA76, §1.2] — that is the value our simulator uses by default (the
informal 9.81 is deliberately **not** used, so the atmosphere model is
self-consistent with the published tables). The conversion between
real (geometric) altitude `h` and geopotential altitude `H` is exact
[COESA76, eq. 19]:

```
H(h) = (Re / (Re + h)) * h          (4)
```

with `Re = 6378137 m` (WGS-84 equatorial radius, [NASA-EF]). This comes
from requiring the potential energy of a unit mass to be equal in the
fictional uniform field (`g0 dH`) and in the real inverse-square field
(`g(h) dh`), with

```
g(h) = g0 * (Re / (Re + h))^2       (5)
```

which is Newton's law of gravitation for a spherically averaged Earth
(1/137 flattening correction ignored — 0.1% class error, below the
model's own accuracy).

**The simulator stores geometric altitude** `h` (what an altimeter
reports) and converts to `H` inside the atmosphere model via the exact
(4). At h = 100 km the small-height approximation `H ≈ h` would err by
1.6%; the exact form keeps the atmosphere valid to the mesopause.

### 2.3 Temperature profile

[COESA76, Table 4] divides the atmosphere into layers; inside each
layer the temperature is *linear* in H:

```
T(H) = Tb + L * (H - Hb)            (6)
```

Layer constants (verbatim from [COESA76, Table 4]; lapse rates converted
from K/km to K/m):

| Hb [km] | Tb [K] | pb [Pa] | L [K/km] | Name |
|---|---|---|---|---|
| 0 | 288.15 | 101325 | −6.5 | Troposphere |
| 11 | 216.65 | 22632.06 | 0 | Tropopause |
| 20 | 216.65 | 5474.889 | +1.0 | Stratosphere (ozone warming) |
| 32 | 228.65 | 868.0187 | +2.8 | Stratosphere |
| 47 | 270.65 | 110.9063 | 0 | Stratopause |
| 51 | 270.65 | 66.93887 | −2.8 | Mesosphere |
| 71 | 214.65 | 3.956420 | −2.0 | Mesosphere |
| 84.852 | 186.87 | 0.3734 | 0 | Mesopause (model top) |

**Why the model stops at 86 km.** Above the mesopause the air is no
longer gravitationally well-mixed: molecular weight starts dropping
(O2 photodissociation) and the ideal-gas + constant-M assumptions break
[COESA76, §1.2 & Table 6]. This is also irrelevant dynamically: at
86 km the density is ~6.5e-6 kg/m^3, so even at Mach 5 the dynamic
pressure is ~0.02 Pa — drag is many orders of magnitude below any other
force. Above the top, the implementation clamps to mesopause values
(a *conservative* choice for the negligible residual drag).

### 2.4 Pressure and density

Pressure follows from **hydrostatic equilibrium** and the **ideal gas
law**. Hydrostatic equilibrium in geopotential coordinates:

```
dp/dH = -rho * g0                   (7)
```

Ideal gas law with constant molecular weight M = 28.9644 kg/kmol and
R* = 8.31432 J/(mol·K) [COESA76, §1.2], giving the specific gas constant
`R = R*/M = 287.0529 J/(kg·K)`:

```
rho = p / (R * T)                   (8)
```

Substituting (8) and (6) into (7) and integrating across a layer from
its base `(Hb, Tb, pb)`:

* **Gradient layer** (L ≠ 0) — [COESA76, eq. 24]:

  ```
  p(H) = pb * (T(H)/Tb)^(-g0/(R*L))          (9)
  ```

* **Isothermal layer** (L = 0) — [COESA76, eq. 28]; this is the
  classical barometric formula with scale height `R*Tb/g0`:

  ```
  p(H) = pb * exp(-g0*(H-Hb)/(R*Tb))        (10)
  ```

Density then comes from (8). Note (9) and (10) are **exact** integrals
of (7) given the layer-linear temperature (6); the only assumption in
the entire atmosphere model is the piecewise linearity of T itself.

Implementation: `src/atmosphere.cpp`, constants in
`include/constants.h`. The physics verification suite
(`src/test_physics.cpp`) reproduces the published [COESA76] table
values at 0, 5, 11, 20, 32, 47, 51 and 71 km to < 0.5 Pa / < 0.2%.

### 2.5 Speed of sound and Mach number

The speed of sound is **computed from the local temperature**, never
hard-coded (the previous code used a fixed 340 m/s at all altitudes,
which underestimates the true value by up to 15% in the cold
tropopause/mesosphere). Definition adopted by [COESA76, §1.3]:

```
a = sqrt(gamma * R * T),  gamma = 1.4       (11)
```

with `gamma = 7/5` for a diatomic ideal gas. Sea-level check:
`sqrt(1.4 * 287.0529 * 288.15) = 340.29 m/s` (verified by the test
suite). Mach number:

```
M = |v| / a(h)                              (12)
```

### 2.6 Gravity in the trajectory model

Inside the trajectory integrator (as opposed to the atmosphere model,
which uses the exact convention above) gravity is taken constant,
`g = g0`. The true decay (5) reduces g by only 3.0% at h = 100 km and
1.5% at 50 km — a few tens of meters on apogee for a sounding rocket,
well below the uncertainty of the drag model. This trade-off is
documented here deliberately: swapping in (5) is a one-line change in
`Rocket::step` (`- mass * g0 * (Re/(Re+h))^2`) and the simulator's CLI
`-g` flag keeps the model overridable for parametric studies
(planets, parametric sensitivity).

## 3. Rocket model

### 3.1 Mass

```
m(t) = mdry + mf(t)                        (13)
```

### 3.2 Propellant consumption — the specific-fuel-consumption model

```
mdot = sfc * T          [kg/s]             (14)
```

`sfc` is the *specific fuel consumption*: propellant mass burned per
newton of thrust per second. The inverse of `sfc` is exactly the
*effective exhaust velocity* `ve = 1/sfc`, because the ideal thrust
equation for a perfectly expanded nozzle [Sutton, eq. 2-14] is:

```
T = mdot * ve                              (15)
```

Rearranged: `mdot = T / ve = sfc * T`. So the mass model and the thrust
model are mutually consistent — the simulator reproduces the
Tsiolkovsky mass-budget behavior by construction. Default
`sfc = 0.0005 kg/(N·s)` ⇔ `ve = 2000 m/s`, a typical composite
solid propellant [Sutton, Table 11-11].

Analytical consequences (both verified in the test suite):

* Burnout time: `tbo = mf0 / (sfc * T)` (§ Test 4).
* Ideal burnout delta-v (Tsiolkovsky): `Δv = ve * ln(m0/mf)`, from
  which the simulator's vacuum runs can be cross-checked by hand.

### 3.3 Thrust cutoff

The engine fires while `mf > 0` and `T > 0`. On the last partial step,
only the remaining propellant is burned (no negative fuel, no firing an
empty engine — the classic constant-thrust-model artifacts).

## 4. Drag model

### 4.1 Drag force

Standard aerodynamic force convention [Anderson, Ch. 1; Sutton, Ch. 3]:

```
D = q * Cd * A,  q = 0.5 * rho * v^2       (16)
```

with `q` the dynamic pressure, `A` the reference (frontal) area, `rho`
the 1976-Standard density (§2.4). The implementation writes the force
with sign as `-0.5 * rho * Cd * A * v * |v|`: the `v|v|` form (rather
than `v^2`) makes drag oppose the motion **in both ascent and descent**
without any sign branching.

### 4.2 Mach dependence of Cd

Cd of a slender body is a strong function of Mach number. AeroSim uses
the canonical three-regime piecewise model (shape described in
[Sutton, Ch. 3] and every projectile-drag survey; see also
[Anderson, Ch. 11 on wave drag]):

```
M <= 0.8   :  Cd = Cd0                        (17)  subsonic
0.8 < M <= 1.2 : Cd = Cd0 * (1 + 2.5*(M-0.8))  (18)  transonic ramp
M > 1.2    :  Cd = 2*Cd0 * sqrt(1.2/M)         (19)  supersonic decay
```

Physical justification, regime by regime:

* **Subsonic (17).** Below the critical Mach number (~0.8 for slender
  bodies) the flow is subcritical everywhere; pressure drag of a
  streamlined body is nearly independent of Mach. `Cd0` is a
  user-supplied constant (default 0.4, typical of a blunt-nose sounding
  rocket [Sutton, Ch. 3, drag discussion]).

* **Transonic ramp (18).** At M ≈ 0.8 local supersonic zones appear on
  the convex surfaces, terminated by shock waves; wave drag grows
  steeply — wind-tunnel surveys of slender projectiles show Cd roughly
  *doubling* between M = 0.8 and M ≈ 1.05–1.2. We model the ramp as
  linear in M with the slope normalized so Cd reaches **exactly
  2·Cd0 at M = 1.2** (`1 + 2.5*(1.2-0.8) = 2`). Continuity at 0.8 is
  exact (ramp starts at Cd0). The linear ramp slightly smooths the real
  (steeper, near-discontinuous) experimental rise; that is a conscious
  choice to keep the curve monotonic, auditable and shock-free for the
  integrator.

* **Supersonic decay (19).** Once the bow shock is fully attached, the
  wave drag of a slender body decays approximately as `1/sqrt(M)` —
  the hypersonic small-disturbance result for bodies of small surface
  inclination [Anderson, hypersonic similarity discussion]. The
  normalization `sqrt(1.2/M)` makes (19) equal the ramp value at the
  junction M = 1.2, so the complete Cd(M) curve is **C(0)-continuous**
  at both breakpoints (verified by Test 5). Check at M = 3:
  `Cd = 2*Cd0*sqrt(0.4) = 1.26*Cd0`, consistent with published
  supersonic Cd decays toward the hypersonic plateau.

This model is deliberately *parametric and transparent* — it captures
the three dominant physical effects (constancy, wave-drag onset,
wave-drag decay) with a single user parameter `Cd0`, and its every
breakpoint is justified above. For higher fidelity (nose-cone shape,
base drag, Reynolds effects) the `Cd(M)` function is a single,
documented seam in `src/rocket.cpp` where a wind-tunnel table or a
semi-empirical model (e.g. a modified Newtonian or a JPL/DRM-style
table) can be dropped in.

## 5. Integrator — semi-implicit (symplectic) Euler

The equations (1)–(3) are stepped with the semi-implicit Euler scheme:

```
a    = F(t, state) / m
v'   = v + a*dt        (kick)     (20)
h'   = h + v'*dt       (drift)   (21)
mf'  = mf - mdot*dt               (22)
```

The single difference from explicit Euler — position is updated with
the **new** velocity — makes the scheme *symplectic* (it is the
kick–drift composition, the simplest member of the leapfrog/Verlet
family [Hairer, §II.16]). Properties that matter for a rocket:

* **Explicit Euler adds energy** on every step for restoring-force
  systems; a descent to terminal velocity then never stabilizes and
  apogee estimates drift with `dt`. The semi-implicit form exactly
  conserves a *shadow* Hamiltonian near the true one, so its energy
  error is bounded at O(dt) instead of accumulating [Hairer, Ch. V].
* Same cost per step as explicit Euler.
* Local error O(dt^2), global error O(dt) — first order. Default
  `dt = 0.1 s` keeps the global error orders of magnitude below the
  drag-model uncertainty for typical sounding-rocket dynamics
  (dynamical time scales of 10–100 s).

**Verified, not asserted**: the physics suite runs a vacuum burn +
coast at dt = 1 ms against the closed-form constant-acceleration
solution `v = a·t`, `h = a·t²/2` and the vacuum apogee
`h + v²/(2g)`, all within 1 m (Test 2); and the burnout time against
`tbo = mf0/(sfc·T)` within one step (Test 4).

For 100× better accuracy at the same memory, an RK4 or a
velocity-Verlet variant can replace (20)–(21) in `Rocket::step` — the
seam is documented in the code.

### 5.1 Drag-stability substepping (adaptive, physics-derived)

A fixed `dt` that is accurate during ascent can be **unstable during a
steep re-entry**. The reason is a property of the physics, not of the
code: quadratic drag `D = k·v·|v|` with `k = rho·Cd·A` acts as a damper
whose instantaneous damping *rate* is

```
|dv/dt| / |v| = k*|v| / m        [1/s]           (23)
```

(obtained by dividing `m·dv/dt = -k·v·|v|` by `v`). Any scheme that
evaluates the drag force explicitly — including semi-implicit Euler — is
only stable if the step is shorter than the inverse of that rate:
within one step, drag must not be able to *reverse* the velocity.
Near the ground the dynamic pressure is thousands of times its apogee
value, and the criterion (23) can exceed `1/dt` by orders of magnitude.
Integrating through that with a fixed step produces a classic numerical
artifact: velocity oscillations that grow without bound (Mach 20 at
touchdown), which is precisely what AeroSim exhibited before this fix.

The implementation therefore splits the requested `dt` into `n` equal
substeps, with

```
n = ceil( dt * k*|v|/m / f_safety ),   f_safety = 0.2   (24)
```

i.e. each substep consumes at most 20% of the local drag damping time
scale. This adds **no new physics parameters**: the criterion (23) is
derived from the drag law itself, and the 0.2 safety factor is an
accuracy margin (5× below the literal stability bound), documented in
`include/rocket.h` (`kMaxDragRateFraction`). Cost: a handful of extra
substeps during the last seconds of descent only.

**Verified by the mission output**: with the default rocket the
simulator now reports a maximum Mach of ~2 during ascent and a
subsonic terminal-velocity descent (~140 m/s at sea level, consistent
with `v_term = sqrt(2·m·g/(rho·Cd·A))`), landing softly at t ≈ 203 s,
instead of the pre-fix numerical artifact.

## 6. Landing model

Descent stops when the altitude reaches 0 with negative velocity: the
state is clamped to (h=0, v=0). No parachute model yet; the impact is
fully inelastic. This is the only non-physical event in the simulation
and it is deliberately crude — adding a descent (drogue/main) system
is a clean extension point in `Rocket::step`.

## 7. What this model does *not* include

Kept out of v0.1 on purpose, in decreasing order of impact on
sounding-rocket accuracy:

1. Angle of attack / pitch dynamics (3-D flight, wind).
2. Parachute descent.
3. Thrust curve (the engine is constant-thrust, not a
   pressure-vs-time profile).
4. Reynolds-number effects on Cd (base drag, boundary-layer regime).
5. Variable gravity in the trajectory integrator (§2.4, ~1% at 50 km).
6. Atmosphere above 86 km (§2.3, dynamically negligible).
7. Winds, Coriolis, Earth rotation.

Each of these has its natural seam pointed out in the code where it
should be added.

## 8. Where to find what in the code

| Topic | File |
|---|---|
| Physical constants + provenance | `include/constants.h` |
| Atmosphere layers, hydrostatic derivation | `include/atmosphere.h`, `src/atmosphere.cpp` |
| Cd(M) drag model, force balance, integrator | `include/rocket.h`, `src/rocket.cpp` |
| CLI parameters and validation | `include/config.h`, `src/config.cpp` |
| Simulation loop, termination, CSV format | `src/main.cpp` |
| Physics verification suite | `src/test_physics.cpp` |
| Plotting (no physics recomputed) | `scripts/plot_results.py` |

Run the verification suite:

```bash
make test
```
