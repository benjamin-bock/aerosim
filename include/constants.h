#ifndef AEROSIM_CONSTANTS_H
#define AEROSIM_CONSTANTS_H

/**
 * @file constants.h
 * @brief Physically referenced constants and default mission parameters.
 *
 * Every constant below carries the exact provenance of its numerical value.
 * Nothing here is a "magic number": each entry states the reference
 * document, the table/section it comes from, and its SI unit.
 *
 * Primary references:
 *  - [COESA76] U.S. Standard Atmosphere 1976, U.S. Government Printing
 *    Office, Washington D.C., 1976. (Joint publication of NOAA, NASA and
 *    the U.S. Air Force.)
 *  - [NIST-CODATA] CODATA 2018 recommended values, as published by NIST.
 */

namespace aerosim {

// ---------------------------------------------------------------------------
// Section 1: Fundamental physical constants
// ---------------------------------------------------------------------------

/**
 * @brief Standard sea-level gravitational acceleration (m/s^2).
 *
 * Source: [COESA76], Section 1.2, "the standard sea-level value of
 * gravitational acceleration" used to define geopotential altitude:
 *     g0 = 9.80665 m/s^2.
 * Note the deliberate difference from the informal 9.81: the 1976
 * Standard Atmosphere is *defined* around g0 = 9.80665, so using any
 * other value would make our atmosphere tables inconsistent with the
 * published temperature/pressure layers.
 */
constexpr double kSeaLevelGravity = 9.80665;  // m/s^2

/**
 * @brief Molar gas constant for the 1976 Standard Atmosphere (J/(mol*K)).
 *
 * Source: [COESA76], Section 1.2: R* = 8.31432 J/(mol*K) (the 1976
 * committee value; NIST CODATA 2018 gives 8.314462618, the difference is
 * < 0.002% and is irrelevant at simulation accuracy).
 */
constexpr double kUniversalGasConstant = 8.31432;  // J/(mol*K)

/**
 * @brief Mean molecular weight of dry air up to 86 km (kg/kmol).
 *
 * Source: [COESA76], Section 1.2 and Table 6: the 1976 atmosphere assumes
 * a gravitationally settled, well-mixed composition with constant
 * M = 28.9644 kg/kmol from sea level up to the mesopause at 86 km.
 * Above 86 km the composition begins to vary with altitude (O2
 * dissociation), which is why our atmosphere model stops there.
 */
constexpr double kAirMolecularWeight = 28.9644;  // kg/kmol

/**
 * @brief Specific gas constant of air (J/(kg*K)).
 *
 * Derived, not fitted: R_specific = R* / M
 *                             = 8.31432 / 28.9644e-3
 *                             = 287.0529 J/(kg*K)
 * This is the value listed in [COESA76] Section 1.2.
 */
constexpr double kSpecificGasConstantAir = 287.0529;  // J/(kg*K)

/**
 * @brief Adiabatic index (ratio of specific heats) of dry air, gamma = cp/cv.
 *
 * Source: ideal diatomic gas at moderate temperature, gamma = 7/5 = 1.4.
 * Used for: (a) the speed of sound a = sqrt(gamma * R * T), and
 * (b) the polytropic exponent 0.190263 in the troposphere pressure
 * law (see atmosphere.cpp). Valid from sea level to ~85 km for the
 * speed-of-sound formula as adopted by [COESA76], Section 1.3.
 */
constexpr double kGammaAir = 1.4;  // dimensionless

/**
 * @brief Earth mean equatorial radius (m).
 *
 * Source: NASA/GSFC Earth Fact Sheet, nominal equatorial radius
 * R_e = 6378.137 km (WGS-84 value). Used only for the optional inverse-
 * square gravity model g(h) = g0 * (R_e/(R_e + h))^2, i.e. Newton's law
 * of universal gravitation applied to a spherically averaged Earth.
 */
constexpr double kEarthRadius = 6378137.0;  // m

/**
 * @brief Sea-level speed of sound is NOT stored as a constant.
 *
 * The speed of sound varies with air temperature and is computed from
 * a(T) = sqrt(gamma * R_specific * T) at the local atmospheric
 * temperature. See atmosphere.h/atmosphere.cpp. At the sea-level
 * standard temperature T0 = 288.15 K this yields the familiar
 * 340.294 m/s, which we expose below only as a documentation
 * reference for sanity checks and reporting.
 */
constexpr double kSeaLevelSpeedOfSound = 340.294;  // m/s (T0 = 288.15 K)

// ---------------------------------------------------------------------------
// Section 2: Default mission parameters (overridable from the command line)
// ---------------------------------------------------------------------------

/**
 * Default rocket parameters are dimensioned to resemble a small
 * solid-motor sounding rocket. They are starting points for
 * experimentation, not a real vehicle.
 */

constexpr double kDefaultDryMass = 50.0;    ///< Dry (structural) mass, kg.
constexpr double kDefaultThrust = 4000.0;  ///< Constant engine thrust, N.
constexpr double kDefaultFuelMass = 100.0; ///< Initial propellant mass, kg.

/**
 * Default integration time step (s). Must satisfy the stability and
 * accuracy discussion in docs/PHYSICS.md, "Integrator": with the
 * semi-implicit (symplectic) Euler scheme, dt must remain small compared
 * to the shortest dynamical time scale of the system, here the thrust
 * phase duration (tens of seconds). 0.1 s keeps the discretization error
 * of order O(dt) well below model uncertainty.
 */
constexpr double kDefaultTimeStep = 0.1;  // s

/**
 * Default specific fuel consumption (kg of propellant per N of thrust
 * per second). Equivalently the inverse of the effective exhaust
 * velocity: mdot = thrust * sfc, and v_e = 1/sfc.
 * The default 0.0005 kg/(N*s) corresponds to an effective exhaust
 * velocity v_e = 2000 m/s, a typical value for a solid propellant
 * (compare: black powder ~800 m/s, composite AP/Al propellant
 * 2000-2800 m/s, see Sutton & Biblarz, "Rocket Propulsion Elements",
 * 9th ed., Table 11-11).
 */
constexpr double kDefaultSpecificFuelConsumption = 0.0005;  // kg/(N*s)

constexpr double kDefaultFrontalArea = 0.1;  ///< Nose frontal area, m^2.
constexpr double kDefaultDragCoefficient = 0.4; ///< Subsonic Cd, see rocket.cpp.

/**
 * Safety cutoff (s). Prevents an infinite loop if the rocket never
 * returns to the ground (e.g. a constant-thrust engine with an unreason-
 * able fuel load on a trajectory that leaves the valid atmosphere range).
 */
constexpr double kMaxSimulationTime = 600.0;  // s

}  // namespace aerosim

#endif  // AEROSIM_CONSTANTS_H
