#include "atmosphere.h"

#include <algorithm>
#include <cmath>

namespace aerosim {

// ---------------------------------------------------------------------------
// Geometric <-> geopotential altitude
// ---------------------------------------------------------------------------
//
// WHY TWO ALTITUDES EXIST
// -----------------------
// The 1976 Standard Atmosphere ([COESA76], Section 1.2) is built by
// pretending gravity is constant (g0 = 9.80665 m/s^2) at all altitudes.
// To keep the physics consistent under that fiction, altitudes are
// measured in "geopotential" meters H, defined so that a parcel moved
// from H1 to H2 gains the same potential energy as it would in reality
// moving between the corresponding geometric altitudes h1, h2.
//
// Real gravity at geometric altitude h (Newton, spherical Earth):
//     g(h) = g0 * (Re / (Re + h))^2
// The geopotential altitude is then defined ([COESA76] eq. 19) as
//     H(h) = (Re / (Re + h)) * h
// which satisfies g0 * dH = g(h) * dh exactly. We use this exact
// conversion, no small-height approximation, so the model stays valid
// even for extreme sounding-rocket apogees (a 100 km apogee differs from
// its small-height approximation by ~1.6%).
//
double Atmosphere::geometricToGeopotential(double geometricAltitude) const {
    if (geometricAltitude <= 0.0) {
        return std::max(geometricAltitude, 0.0);
    }
    return kEarthRadius * geometricAltitude / (kEarthRadius + geometricAltitude);
}

// ---------------------------------------------------------------------------
// Layer table: US Standard Atmosphere 1976, [COESA76] Table 4.
// ---------------------------------------------------------------------------
// Each row: {H_base [m], T_base [K], p_base [Pa], lapse L [K/m]}.
// These are the published values; converting lapse rates from K/km to
// K/m introduces no rounding beyond double precision.
//
// Layer boundaries at 11, 20, 32, 47, 51 and 71 km geopotential were
// chosen by the COESA committee to match measured mean temperature
// profiles; they are not adjustable parameters of this simulation.
Atmosphere::Atmosphere()
    : layers_{
          {0.0, 288.15, 101325.0, -0.0065},   // Troposphere: T falls 6.5 K/km
          {11000.0, 216.65, 22632.06, 0.0},   // Tropopause: isothermal
          {20000.0, 216.65, 5474.889, 0.001},  // Stratosphere: warms +1 K/km (ozone)
          {32000.0, 228.65, 868.0187, 0.0028}, // Warming continues, +2.8 K/km
          {47000.0, 270.65, 110.9063, 0.0},    // Stratopause: isothermal
          {51000.0, 270.65, 66.93887, -0.0028}, // Mesosphere: cools -2.8 K/km
          {71000.0, 214.65, 3.956420, -0.002}, // Lower mesosphere: -2.0 K/km
          {84852.0, 186.87, 0.3734, 0.0},      // Mesopause (top of model)
      } {}

size_t Atmosphere::layerIndex(double geopotential) const {
    // Walk from the top down: the first layer whose base is below the
    // requested altitude is the containing layer. Beyond the top we
    // clamp to the last layer, so pressure/density/temperature simply
    // hold their mesopause values; the resulting force error above 86 km
    // is many orders of magnitude below any other model error.
    for (size_t i = layers_.size(); i-- > 0;) {
        if (geopotential >= layers_[i].baseGeopotential) {
            return i;
        }
    }
    return 0;  // Below sea level: use sea-level layer (clamped).
}

// ---------------------------------------------------------------------------
// Temperature: [COESA76] eq. 23 (gradient layers) / eq. 26 (isothermal).
//   T = T_base + L * (H - H_base)
// In isothermal layers L = 0 and the same formula trivially applies.
double Atmosphere::temperature(double geometricAltitude) const {
    const double H = geometricToGeopotential(geometricAltitude);
    const AtmosphericLayer& layer = layers_[layerIndex(H)];
    return layer.baseTemperature + layer.lapseRate * (H - layer.baseGeopotential);
}

// ---------------------------------------------------------------------------
// Pressure: hydrostatic equilibrium + ideal gas law.
//
// DERIVATION (see docs/PHYSICS.md for the full annotated version):
//   Hydrostatic equilibrium in geopotential coordinates:
//       dp/dH = -rho * g0                          ... (1)
//   Ideal gas law:
//       rho = p / (R_spec * T)                     ... (2)
//   Layer-linear temperature:
//       T(H) = T_b + L * (H - H_b)                 ... (3)
//
//   Substituting (2)+(3) into (1):
//       dp/p = -g0 / (R_spec * (T_b + L*(H-H_b))) dH
//   Integrating from the layer base to H:
//
//   Gradient layer (L != 0):
//       p = p_b * (T/T_b)^(-g0/(R_spec*L))
//   Isothermal layer (L == 0):
//       p = p_b * exp(-g0*(H-H_b) / (R_spec*T_b))
//
//   These are EXACT integrals of (1) given (3); the only assumption is
//   the [COESA76] piecewise-linear temperature profile itself.
//
double Atmosphere::pressure(double geometricAltitude) const {
    const double H = geometricToGeopotential(geometricAltitude);
    const AtmosphericLayer& layer = layers_[layerIndex(H)];
    const double dH = H - layer.baseGeopotential;
    const double T = layer.baseTemperature + layer.lapseRate * dH;

    if (layer.lapseRate == 0.0) {
        // Isothermal: barometric formula with constant T.
        // g0/(R_spec*T_b) is the inverse of the familiar "scale height".
        return layer.basePressure *
               std::exp(-kSeaLevelGravity * dH / (kSpecificGasConstantAir * layer.baseTemperature));
    }
    // Gradient layer: polytropic relation.
    // Exponent -g0/(R_spec*L): for the troposphere L<0 so the exponent is
    // positive and pressure decreases with altitude, as expected.
    const double exponent = -kSeaLevelGravity / (kSpecificGasConstantAir * layer.lapseRate);
    return layer.basePressure * std::pow(T / layer.baseTemperature, exponent);
}

// ---------------------------------------------------------------------------
// Density: ideal gas law with the local p and T.
//   rho = p / (R_spec * T)          [COESA76] eq. 12
// Note: computing rho from p and T (rather than integrating its own
// hydrostatic equation) guarantees thermodynamic self-consistency.
double Atmosphere::density(double geometricAltitude) const {
    const double p = pressure(geometricAltitude);
    const double T = temperature(geometricAltitude);
    return p / (kSpecificGasConstantAir * T);
}

// ---------------------------------------------------------------------------
// Speed of sound: [COESA76] Section 1.3, adopted definition
//     a = sqrt(gamma * R_spec * T)
// gamma = 1.4 (diatomic ideal gas) is the value adopted by the standard.
// Sanity check at sea level: sqrt(1.4 * 287.0529 * 288.15) = 340.29 m/s.
double Atmosphere::speedOfSound(double geometricAltitude) const {
    return std::sqrt(kGammaAir * kSpecificGasConstantAir * temperature(geometricAltitude));
}

// ---------------------------------------------------------------------------
// Mach number: dimensionless ratio V/a. Sign of V is irrelevant to the
// magnitude, hence the std::fabs.
double Atmosphere::machNumber(double geometricAltitude, double speed) const {
    return std::fabs(speed) / speedOfSound(geometricAltitude);
}

// ---------------------------------------------------------------------------
// Dynamic pressure: q = 0.5 * rho * V^2.
// This is the definition used in the standard aerodynamic coefficient
// convention (Sutton & Biblarz, "Rocket Propulsion Elements", 9th ed.,
// Section 3.1; Anderson, "Fundamentals of Aerodynamics", Chapter 1):
//     Drag force D = q * Cd * A  <=>  Cd = D / (q * A)
double Atmosphere::dynamicPressure(double geometricAltitude, double speed) const {
    return 0.5 * density(geometricAltitude) * speed * speed;
}

}  // namespace aerosim
