#ifndef AEROSIM_ATMOSPHERE_H
#define AEROSIM_ATMOSPHERE_H

#include <vector>

#include "constants.h"

/**
 * @file atmosphere.h
 * @brief Piecewise-linear temperature model and layered US Standard
 *        Atmosphere 1976, plus the utility functions derived from it
 *        (density, dynamic pressure, speed of sound, Mach number).
 *
 * MODEL SUMMARY (full derivation and justifications in docs/PHYSICS.md):
 *
 *  1. Temperature profile T(h):
 *     The 1976 Standard Atmosphere ([COESA76], Figure 4 and Table 4)
 *     divides the atmosphere into layers. Inside each layer the
 *     temperature is an exact straight line in GEOPOTENTIAL altitude:
 *
 *         T(H) = T_layer_base + L_layer * (H - H_layer_base)
 *
 *     where L is the "lapse rate" (negative in the troposphere, zero in
 *     the isothermal layers, positive in inversion layers). Because we
 *     simulate at most a few hundred km of vertical flight, we implement
 *     the seven layers from sea level to 86 km (geopotential) — the
 *     range over which air is well mixed and the ideal-gas law with a
 *     constant molecular weight holds ([COESA76], Section 1.2).
 *
 *  2. Pressure profile p(h):
 *     Derived by integrating hydrostatic equilibrium + ideal gas law.
 *     Hydrostatic equilibrium:  dp = -rho * g0 * dH   (geopotential)
 *     Ideal gas law:            rho = p / (R_spec * T)
 *     Substituting and integrating over a layer with lapse rate L:
 *
 *       L != 0 (gradient layer):
 *           p(h) = p_base * (T/T_base)^(-g0/(R_spec*L))
 *     L == 0 (isothermal layer):
 *           p(h) = p_base * exp(-(g0/(R_spec*T_base)) * (H - H_base))
 *
 *     These are the exact closed-form integrals; no approximation is
 *     made beyond the layer-linear temperature assumption itself.
 *
 *  3. Density: rho = p / (R_spec * T)   (ideal gas law).
 *
 *  4. Speed of sound: a = sqrt(gamma * R_spec * T)
 *     ([COESA76], Section 1.3: a = (gamma R T)^(1/2) is the *adopted*
 *     definition of the speed of sound in the standard).
 */

namespace aerosim {

/**
 * @struct AtmosphericLayer
 * @brief One layer of the piecewise-linear 1976 Standard Atmosphere.
 *
 * A layer is defined by:
 *   - baseGeopotential: geopotential altitude of its lower boundary (m)
 *   - baseTemperature:  temperature at that boundary (K)
 *   - basePressure:    pressure at that boundary (Pa)
 *   - lapseRate:       dT/dH within the layer (K/m); 0 for isothermal
 *
 * The base values are taken verbatim from [COESA76], Table 4
 * ("Standard Atmosphere, 1976, up to 86 kilometers"), so the simulation
 * reproduces the published tables exactly at every layer boundary.
 */
struct AtmosphericLayer {
    double baseGeopotential;  // m
    double baseTemperature;   // K
    double basePressure;      // Pa
    double lapseRate;         // K/m
};

/**
 * @class Atmosphere
 * @brief Computes temperature, pressure, density and speed of sound as
 *        functions of geometric altitude, following the US Standard
 *        Atmosphere 1976 up to 86 km geopotential.
 *
 * Altitude convention used throughout AeroSim:
 *   - The rocket state stores GEOMETRIC altitude h (m above mean sea
 *     level), because that is what a real altimeter/radar reports.
 *   - The 1976 Standard Atmosphere is defined in GEOPOTENTIAL altitude
 *     H = g0/g(h) * h, which wraps the altitudes into a uniform gravity
 *     field. We perform the exact conversion (see atmosphere.cpp).
 */
class Atmosphere {
public:
    /**
     * @brief Construct the standard 1976 atmosphere.
     *
     * The layer table is initialized with the published base values:
     *   H=0 km:      T=288.15 K,  p=101325 Pa,  L=-6.5 K/km  (troposphere)
     *   H=11 km:     T=216.65 K,  p=22632.06 Pa, L=0          (tropopause)
     *   H=20 km:     T=216.65 K,  p=5474.889 Pa, L=+1.0 K/km (stratosphere)
     *   H=32 km:     T=228.65 K,  p=868.0187 Pa, L=+2.8 K/km
     *   H=47 km:     T=270.65 K,  p=110.9063 Pa, L=0
     *   H=51 km:     T=270.65 K,  p=66.93887 Pa, L=-2.8 K/km (mesosphere)
     *   H=71 km:     T=214.65 K,  p=3.956420 Pa, L=-2.0 K/km
     *   H=84.852 km: top of the valid range (mesopause).
     * All numbers: [COESA76], Table 4. Beyond the last layer the model
     * clamps and returns the top-of-range values (see .cpp), which for a
     * ballistic sounding rocket is acceptable because aerodynamic forces
     * above ~86 km are already utterly negligible (dynamic pressure at
     * 86 km is ~1e-6 % of its sea-level magnitude for the same speed).
     */
    Atmosphere();

    // --- Primary state functions (all take GEOMETRIC altitude, m) ---

    /// Temperature at altitude (K). Layer-linear, [COESA76] Table 4.
    double temperature(double geometricAltitude) const;

    /// Pressure at altitude (Pa). See hydrostatic derivation above.
    double pressure(double geometricAltitude) const;

    /// Density at altitude (kg/m^3). Ideal gas law rho = p/(R_spec*T).
    double density(double geometricAltitude) const;

    /// Speed of sound at altitude (m/s). a = sqrt(gamma*R_spec*T),
    /// [COESA76] Section 1.3.
    double speedOfSound(double geometricAltitude) const;

    /**
     * @brief Mach number at a given altitude and speed.
     * Mach = V / a(h). Both inputs SI; result dimensionless.
     */
    double machNumber(double geometricAltitude, double speed) const;

    /**
     * @brief Dynamic pressure q at altitude (Pa).
     * q = 0.5 * rho(h) * V^2. This is the exact quantity appearing in the
     * aerodynamic force coefficients definition
     * F = q * Cd * A, and is the same "q" used to define the dimension-
     * less Karman-style flight regimes in reporting.
     */
    double dynamicPressure(double geometricAltitude, double speed) const;

    /// Exact geometric -> geopotential altitude conversion (public so
    /// the verification suite can invert it): H = (Re/(Re+h)) * h.
    /// Full derivation in the .cpp and docs/PHYSICS.md section 2.2.
    double geometricToGeopotential(double geometricAltitude) const;

private:
    /// Layer table from sea level up (units as in AtmosphericLayer).
    std::vector<AtmosphericLayer> layers_;

    /// Index of the layer containing the given geopotential altitude,
    /// clamped into the valid range [0, layers_.size()-1].
    size_t layerIndex(double geopotential) const;
};

}  // namespace aerosim

#endif  // AEROSIM_ATMOSPHERE_H
