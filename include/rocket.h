#ifndef AEROSIM_ROCKET_H
#define AEROSIM_ROCKET_H

#include "atmosphere.h"

/**
 * @file rocket.h
 * @brief One-dimensional (vertical) flight dynamics of a thrusting,
 *        drag-affected rocket with time-varying mass.
 *
 * STATE AND FORCES
 * ----------------
 * The rocket is modeled as a point mass moving along the vertical axis.
 * State variables (all SI):
 *   - position: geometric altitude h [m]
 *   - velocity: vertical velocity v [m/s], positive = upward
 *   - fuel:     remaining propellant mass [kg]
 *
 * Forces along the vertical axis (positive = up), applied in this order
 * at every integration step (see docs/PHYSICS.md for the full picture):
 *
 *   Thrust T [N]      : constant engine thrust while fuel remains.
 *                       Aligned with the velocity axis; for a vertically
 *                       launched rocket without angle of attack this is
 *                       purely vertical.
 *   Drag D [N]        : D = 0.5 * rho * Cd(M) * A * |v| * v
 *                       The |v|*v form (rather than v^2) makes the drag
 *                       force oppose the motion in both ascent and
 *                       descent; rho comes from the US Standard
 *                       Atmosphere 1976 (atmosphere.h), Cd from the
 *                       Mach-dependent model below.
 *   Gravity W [N]     : W = m * g, always directed downward.
 *
 * MASS MODEL
 * ----------
 * m(t) = dryMass + fuel(t), and the engine consumes propellant at
 *     mdot = sfc * T
 * i.e. kilograms burned per second, proportional to the thrust. The
 * equivalent effective exhaust velocity is v_e = 1/sfc, which is exactly
 * the Tsiolkovsky thrust equation T = mdot * v_e rearranged. So the mass
 * model is consistent with the ideal rocket equation, and the sim can
 * reproduce the classic delta-v figures (see docs/PHYSICS.md).
 *
 * DRAG COEFFICIENT MODEL Cd(M)
 * ----------------------------
 * The drag coefficient of a slender body is a strong function of the
 * Mach number M. We implement a standard three-regime piecewise model
 * (regimes described e.g. in Sutton & Biblarz, "Rocket Propulsion
 * Elements", 9th ed., Chapter 3, and Anderson, "Modern Compressible
 * Flow", Chapter 9). Full curve, formulas and references: rocket.cpp
 * and docs/PHYSICS.md.
 *
 *   Subsonic  (M < 0.8) : Cd is constant = Cd0. Below Mach ~0.8 the flow
 *                         is subcritical; Cd of a streamlined body is
//                        roughly constant.
 *   Transonic (0.8-1.2): Cd rises steeply, reaching 2x the subsonic
 *                         value at M = 1.2 for a sharp-nosed slender
 *                         body. The physical cause is
 *                         the formation of supersonic zones terminated
 *                         by shock waves (wave drag), which appears as
 *                         a near-discontinuous jump in real wind-tunnel
 *                         data. We model it as a linear ramp.
 *   Supersonic (M > 1.2): Cd falls off approximately as 1/sqrt(M)
 *                         ("hypersonic-like" decay of wave drag for
 *                         slender bodies; see flows with small surface
 *                         inclination). We implement Cd ~ 2*Cd0/sqrt(M)
 *                         which equals the transonic peak value at
 *                         M = 1.2 by construction (see rocket.cpp).
 *
 * INTEGRATOR
 * ----------
 * Semi-implicit (symplectic) Euler. See step() and docs/PHYSICS.md for
 * the derivation of the update equations and why this scheme is chosen
 * over explicit Euler.
 */

namespace aerosim {

/**
 * @brief Stability cap for the drag-damping substepping (dimensionless).
 *
 * The drag damping rate of a falling rocket is k*|v|/m with
 * k = rho*Cd*A (derived in rocket.cpp and docs/PHYSICS.md, section 5:
 * from m*dv/dt = -k*v*|v|, the velocity decay rate is k*|v|/m). Any
 * scheme explicit in the drag force is only stable if the time step is
 * smaller than the inverse of that rate. We enforce that each internal
 * substep consumes at most kMaxDragRateFraction of the damping time
 * scale. 0.2 is a conservative engineering margin (5x below the literal
 * stability bound dt < tau): it keeps the re-entry phase accurate, not
 * merely non-diverging, at the cost of a handful of extra substeps in
 * the last seconds of descent.
 */
constexpr double kMaxDragRateFraction = 0.2;

/// Flight phases, derived from state rather than stored, so that they
/// can never disagree with the physical state.
enum class FlightPhase {
    PreLaunch,  ///< On the pad, t = 0 (reported for completeness).
    PoweredAscent,  ///< Engine burning, moving upward.
    CoastingAscent, ///< Engine spent, still moving upward.
    Descent,    ///< Falling back (velocity negative).
    Landed,     ///< Back at ground level.
};

/// Snapshot of the rocket state at one instant, used for logging and
/// for the Python plotting pipeline. Pure data, no behavior.
struct Telemetry {
    double time = 0.0;             ///< Simulation time [s]
    double altitude = 0.0;         ///< Geometric altitude [m]
    double velocity = 0.0;         ///< Vertical velocity [m/s], up = +
    double acceleration = 0.0;     ///< Vertical acceleration [m/s^2]
    double mass = 0.0;             ///< Total (dry + fuel) mass [kg]
    double fuel = 0.0;             ///< Remaining propellant [kg]
    double mach = 0.0;             ///< Local Mach number [-]
    double drag = 0.0;             ///< Drag force magnitude [N]
    double thrust = 0.0;           ///< Current thrust (0 after burnout) [N]
    double dynamicPressure = 0.0;  ///< q = 0.5*rho*v^2 [Pa]
    double density = 0.0;          ///< Local air density [kg/m^3]
    double dragCoefficient = 0.0;  ///< Cd evaluated at the local Mach [-]
    double speedOfSound = 0.0;     ///< Local speed of sound [m/s]
};

class Rocket {
public:
    /**
     * @brief Build a rocket from its physical description.
     * @param dryMass      Structural mass, kg (> 0)
     * @param thrust       Engine thrust, N (>= 0)
     * @param initialFuel  Propellant mass at ignition, kg (>= 0)
     * @param area         Reference (frontal) area for drag, m^2 (> 0)
     * @param dragCoeff    Subsonic drag coefficient Cd0 (> 0)
     * @param gravity      Gravitational acceleration, m/s^2 (> 0)
     * @param specificFuelConsumption  Propellant burned per N of thrust
     *                                 per second, kg/(N*s) (>= 0)
     */
    Rocket(double dryMass, double thrust, double initialFuel, double area,
           double dragCoeff, double gravity, double specificFuelConsumption);

    /**
     * @brief Advance the state by dt seconds using the semi-implicit
     *        (symplectic) Euler scheme.
     *
     * UPDATE EQUATIONS (see docs/PHYSICS.md, "Integrator"):
     *   a  = F_net(t, state) / m
     *   v' = v + a * dt        (velocity first: "implicit" in velocity)
     *   x' = x + v' * dt       (position uses the NEW velocity)
     *   fuel' = fuel - mdot*dt (burn, clamped at zero)
     * This ordering is what makes the integrator symplectic: it
     * conserves a modified energy and does not gain energy
     * artificially on descent, unlike explicit Euler, which
     * systematically adds energy for oscillatory/stiff systems.
     */
    void step(double dt, const Atmosphere& atmosphere);

    /// Current derived flight phase (see FlightPhase).
    FlightPhase phase() const;

    /// Build a full telemetry snapshot of the current state.
    Telemetry telemetry(double time, const Atmosphere& atmosphere) const;

    // --- Getters (SI units; see member docs in rocket.cpp) ---
    double altitude() const { return altitude_; }
    double velocity() const { return velocity_; }
    double acceleration() const { return lastAcceleration_; }
    double fuel() const { return fuel_; }
    double totalMass() const { return dryMass_ + fuel_; }
    bool engineFiring() const { return fuel_ > 0.0 && thrust_ > 0.0; }
    double burnoutTime() const { return burnoutTime_; }

private:
    /**
     * @brief Mach-dependent drag coefficient Cd(M), dimensionless.
     *
     * Three-regime piecewise model (see file header for the physical
     * discussion). Exact formulas in rocket.cpp.
     */
    double dragCoefficient(double mach) const;

    // -- Fixed vehicle parameters (set at construction) --
    const double dryMass_;   ///< Structural mass [kg]
    const double thrust_;    ///< Engine thrust [N]
    const double area_;      ///< Reference area [m^2]
    const double cdSubsonic_; ///< Subsonic drag coefficient [-]
    const double gravity_;   ///< Gravitational acceleration [m/s^2]
    const double sfc_;       ///< Specific fuel consumption [kg/(N*s)]

    // -- Time-varying state --
    double altitude_;     ///< Geometric altitude [m]
    double velocity_;     ///< Vertical velocity [m/s]
    double fuel_;         ///< Remaining propellant [kg]
    double burnoutTime_;  ///< Time at which the fuel hit zero [s],
                          ///< negative until it happens.
    double lastAcceleration_ = 0.0; ///< Acceleration of the last step [m/s^2]
    double currentTime_ = 0.0;      ///< Internal mission clock [s]
};

}  // namespace aerosim

#endif  // AEROSIM_ROCKET_H
