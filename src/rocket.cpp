#include "rocket.h"

#include <algorithm>
#include <cmath>

namespace aerosim {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
Rocket::Rocket(double dryMass, double thrust, double initialFuel, double area,
               double dragCoeff, double gravity, double specificFuelConsumption)
    : dryMass_(dryMass),
      thrust_(thrust),
      area_(area),
      cdSubsonic_(dragCoeff),
      gravity_(gravity),
      sfc_(specificFuelConsumption),
      altitude_(0.0),
      velocity_(0.0),
      fuel_(initialFuel),
      burnoutTime_(-1.0) {}

// ---------------------------------------------------------------------------
// Drag coefficient vs Mach number
// ---------------------------------------------------------------------------
//
// MODEL AND PROVENANCE
// --------------------
// The drag coefficient of a slender, sharp-nosed body varies with Mach
// number in a well-documented way. Three regimes are distinguished
// (see, e.g., Sutton & Biblarz, "Rocket Propulsion Elements", 9th ed.,
// Chapter 3; Anderson, "Fundamentals of Aerodynamics", compressible
// drag discussion; and classic transonic wind-tunnel surveys of
// projectile drag):
//
//   M < 0.8 (subsonic):
//     The flow over the body stays fully subcritical. Cd of a
//     streamlined body is nearly constant; we use the user-supplied
//     Cd0 flat value:
//         Cd(M) = Cd0
//
//   0.8 <= M <= 1.2 (transonic):
//     Local supersonic zones form on the body and terminate in shock
//     waves; wave drag appears abruptly. Wind-tunnel data for slender
//     bodies shows Cd roughly doubling between M = 0.8 and its peak
//     near M = 1.05. We model the ramp as LINEAR in M and normalize the
//     slope so that Cd reaches exactly 2*Cd0 at M = 1.2, where the
//     supersonic branch takes over:
//         Cd(M) = Cd0 * (1 + 2.5*(M - 0.8))
//     Check: Cd(1.2) = Cd0 * (1 + 2.5*0.4) = 2*Cd0 exactly, and the
//     curve is continuous at M = 0.8 by construction.
//
//   M > 1.2 (supersonic):
//     For a slender body at small angles of attack, the wave drag of
//     nose cones decays approximately as 1/sqrt(M) once the bow shock
//     is fully attached (hypersonic small-disturbance result for
//     bodies of small surface inclination; see Anderson, "Hypersonic
//     and High-Temperature Gas Dynamics", thin-shock-layer theory).
//     We use:
//         Cd(M) = Cd(1.2) * sqrt(1.2/M) = 2*Cd0 * sqrt(1.2/M)
//     The sqrt(1.2/M) factor makes the branch equal the transonic
//     value at the junction M = 1.2 and decay as 1/sqrt(M) beyond, so
//     the full curve is continuous at both breakpoints. This is
//     verified by the physics test suite (src/test_physics.cpp).
//
double Rocket::dragCoefficient(double mach) const {
    if (mach < 0.8) {
        // Subsonic: constant Cd0.
        return cdSubsonic_;
    }
    if (mach <= 1.2) {
        // Transonic: linear ramp from Cd0 at M=0.8 to 2*Cd0 at M=1.2
        // (slope 2.5: Cd(1.2) = Cd0*(1 + 2.5*0.4) = 2*Cd0 exactly).
        return cdSubsonic_ * (1.0 + 2.5 * (mach - 0.8));
    }
    // Supersonic: 2*Cd0 at M=1.2, decaying as 1/sqrt(M).
    return 2.0 * cdSubsonic_ * std::sqrt(1.2 / mach);
}

// ---------------------------------------------------------------------------
// One integration step: semi-implicit (symplectic) Euler with
// drag-stability substepping
// ---------------------------------------------------------------------------
//
// WHY SEMI-IMPLICIT EULER
// -----------------------
// Explicit Euler updates position with the OLD velocity and velocity with
// the OLD acceleration. For any system with restoring forces this scheme
// ADDS energy at every step. Semi-implicit Euler swaps one update:
//     v' = v + a*dt,  x' = x + v'*dt
// which is the simplest symplectic integrator (the kick-drift composition
// of a leapfrog step; see Hairer, Lubich & Wanner, "Geometric Numerical
// Integration", 2nd ed., Section II.16): its energy error stays bounded
// at O(dt) instead of accumulating. Same cost as explicit Euler.
//
// THE DRAG STABILITY CONDITION (why we substep)
// ----------------------------------------------
// Drag is not a restoring force around a fixed point but a strongly
// non-linear damper: D = 0.5*rho*Cd*A*v*|v|. Its characteristic damping
// rate is
//     tau(v,h) = m / (rho(h)*Cd*A*|v|)      [s]
// obtained by dividing m*dv/dt = -k*v*|v| (k = rho*Cd*A) by v: the
// velocity decay rate is k*|v|/m = 1/tau. Any explicit-in-the-drag-force
// scheme (including semi-implicit Euler) is only stable when
//     dt < tau
// Physically: within one step, drag must not be able to reverse the
// velocity. During a steep re-entry the dynamic pressure grows by
// orders of magnitude in seconds, so a FIXED dt that is fine at apogee
// can be violently unstable near the ground: with dt = 0.1 s at
// v = 700 m/s near sea level, a single step can overshoot the terminal
// velocity and flip v, and the solution then oscillates and grows
// (the classic numerical "drag instability", not a physics result).
//
// FIX: adaptive substepping on the physics-derived criterion. We split
// the user-requested dt into n equal substeps, with
//     n = ceil(dt * rho*Cd*A*|v| / m / kMaxDragRateFraction)
// i.e. we cap the fraction of the damping time tau consumed by one
// substep at a safety factor (kMaxDragRateFraction, see header). This
// adds ZERO parameters to the physics: the criterion is derived from
// the damping rate itself. Typical cost: a handful of substeps per
// step during the last seconds of descent; no cost in the ascent,
// coast, and vacuum phases.
//
// FORCE BALANCE (positive = up), evaluated at the CURRENT state:
//     F_net = T_active - D - m*g
//       T_active = T while fuel remains, else 0 (engine cutoff).
//       D = 0.5 * rho(h) * Cd(M) * A * v * |v|
//           The v*|v| product (rather than v^2) is essential: it makes
//           drag always OPPOSE the motion, i.e. on descent (v<0) the
//           drag term is an upward (decelerating) force automatically.
//       g is taken constant (the inverse-square decay changes g by
//           <3% below 100 km; see docs/PHYSICS.md section 2.4 for the
//           trade-off and how to change it).
//
// MASS AND FUEL
// --------------
// While fuel remains, propellant is burned at
//     mdot = sfc * T            (kg/s)
// the definition of specific fuel consumption; equivalently, with the
// effective exhaust velocity v_e = 1/sfc, this is T = mdot * v_e, the
// ideal thrust equation (Sutton & Biblarz, "Rocket Propulsion Elements",
// 9th ed., eq. 2-14). The burn is applied proportionally across the
// substeps so the total propellant spent is exactly mdot*dt.
//
void Rocket::step(double dt, const Atmosphere& atmosphere) {
    // --- 1. Local atmospheric conditions (for substep count only) ---
    // Density is evaluated inside atmosphere.dynamicPressure(); see
    // atmosphere.cpp for the q = 0.5*rho*V^2 definition.
    const double rho0 = atmosphere.density(altitude_);
    const double mach0 = atmosphere.machNumber(altitude_, velocity_);
    const double cd0 = dragCoefficient(mach0);

    // --- 2. Drag-stability substepping (see the derivation above) ---
    // Damping rate k*|v|/m with k = rho*Cd*A; each substep must consume
    // at most kMaxDragRateFraction of the damping time scale.
    const double mass0 = dryMass_ + fuel_;
    const double dampingRate = rho0 * cd0 * area_ * std::fabs(velocity_) / mass0;
    const size_t substeps = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(dt * dampingRate / kMaxDragRateFraction)));
    const double h = dt / static_cast<double>(substeps);

    // --- 3. Propellant to burn over the full step, then split evenly ---
    // The engine fires only while there is propellant left. When the
    // remaining fuel is less than one full step's worth, we burn only
    // what remains. This avoids the classic artifacts of constant-
    // thrust models: negative fuel mass and firing an empty engine.
    double propellantToBurn = 0.0;
    if (fuel_ > 0.0 && thrust_ > 0.0) {
        propellantToBurn = std::min(fuel_, thrust_ * sfc_ * dt);
    }

    // --- 4. Substep loop: semi-implicit Euler (see header) ---
    for (size_t n = 0; n < substeps; ++n) {
        const double subMach = atmosphere.machNumber(altitude_, velocity_);
        const double subCd = dragCoefficient(subMach);
        // SIGNED drag force: D = 0.5*rho*Cd*A*v*|v|. The v*|v| product
        // (rather than v^2) carries the sign so that drag always
        // OPPOSES the motion: D < 0 on descent (upward force), and the
        // -D term in F_net then correctly decelerates the fall.
        const double dragForce =
            atmosphere.dynamicPressure(altitude_, velocity_) * subCd * area_ *
            (velocity_ < 0.0 ? -1.0 : 1.0);
        const double subMass = dryMass_ + fuel_;

        // Thrust for this substep: full thrust while propellant remains.
        double activeThrust = 0.0;
        if (fuel_ > 0.0 && thrust_ > 0.0) {
            const double burn = propellantToBurn / static_cast<double>(substeps);
            fuel_ -= burn;
            if (fuel_ <= 0.0) {
                fuel_ = 0.0;
                burnoutTime_ = currentTime_ + dt * (static_cast<double>(n) + 1.0) /
                                              static_cast<double>(substeps);
            }
            activeThrust = thrust_;
        }

        // Newton's second law for the (variable-mass) point rocket.
        const double netForce = activeThrust - dragForce - subMass * gravity_;
        lastAcceleration_ = netForce / subMass;

        // Kick-drift update (symplectic ordering).
        velocity_ += lastAcceleration_ * h;  // kick
        altitude_ += velocity_ * h;          // drift with NEW velocity

        // Ground clamp: the rocket cannot penetrate the launch pad.
        if (altitude_ <= 0.0 && velocity_ < 0.0) {
            altitude_ = 0.0;
            velocity_ = 0.0;
            break;
        }
    }

    currentTime_ += dt;
}

// ---------------------------------------------------------------------------
// Phase derivation: purely a function of state, never stored, so it
// cannot desynchronize from the physics.
FlightPhase Rocket::phase() const {
    if (altitude_ <= 0.0 && velocity_ == 0.0 && currentTime_ == 0.0) {
        return FlightPhase::PreLaunch;
    }
    if (altitude_ <= 0.0 && velocity_ == 0.0) {
        return FlightPhase::Landed;
    }
    if (velocity_ >= 0.0) {
        return engineFiring() ? FlightPhase::PoweredAscent : FlightPhase::CoastingAscent;
    }
    return FlightPhase::Descent;
}

// ---------------------------------------------------------------------------
// Telemetry snapshot: everything the CSV/plot pipeline consumes, in SI.
Telemetry Rocket::telemetry(double time, const Atmosphere& atmosphere) const {
    Telemetry t;
    t.time = time;
    t.altitude = altitude_;
    t.velocity = velocity_;
    t.acceleration = lastAcceleration_;
    t.mass = totalMass();
    t.fuel = fuel_;
    t.mach = atmosphere.machNumber(altitude_, velocity_);
    t.dragCoefficient = dragCoefficient(t.mach);
    t.drag = atmosphere.dynamicPressure(altitude_, velocity_) * t.dragCoefficient * area_;
    t.thrust = engineFiring() ? thrust_ : 0.0;
    t.dynamicPressure = atmosphere.dynamicPressure(altitude_, velocity_);
    t.density = atmosphere.density(altitude_);
    t.speedOfSound = atmosphere.speedOfSound(altitude_);
    return t;
}

}  // namespace aerosim
