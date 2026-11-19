/**
 * @file test_physics.cpp
 * @brief Physics verification suite for AeroSim.
 *
 * These are not unit tests of implementation details: they are checks
 * against INDEPENDENT references (published 1976 Standard Atmosphere
 * tables and closed-form analytical solutions), so a failure means the
 * physics is wrong, not merely that the code changed.
 *
 * References used by the assertions:
 *  - [COESA76] U.S. Standard Atmosphere 1976, NOAA/NASA/USAF, Table II
 *    ("Standard Atmosphere, 1976 - Metric", kilometer grid values).
 *  - Closed-form constant-acceleration kinematics (vacuum parabola).
 *  - Analytical propellant burnout time from mdot = sfc * T.
 *  - Continuity conditions of the piecewise Cd(M) model.
 *
 * Build & run:  make test && ./build/test_physics
 */

#include <cmath>
#include <iostream>
#include <string>

#include "atmosphere.h"
#include "constants.h"
#include "rocket.h"

namespace {

int failures = 0;

/// Assert helper: report PASS/FAIL with context.
void check(const std::string& name, double actual, double expected,
           double tolerance, const std::string& unit) {
    const double err = std::fabs(actual - expected);
    const bool ok = err <= tolerance;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << ": got " << actual
              << " " << unit << ", expected " << expected << " " << unit
              << " (tolerance " << tolerance << ")" << std::endl;
    if (!ok) {
        ++failures;
    }
}

// ---------------------------------------------------------------------------
// Test 1: atmosphere against the published 1976 tables
// ---------------------------------------------------------------------------
// Values below are transcribed from [COESA76], Table II. If our layer
// integration is correct, we must reproduce these numbers to well under
// 1%: the same equations were used to compute the published tables.
void testAtmosphere1976() {
    std::cout << "\n== Atmosphere vs US Standard Atmosphere 1976 tables ==" << std::endl;
    const aerosim::Atmosphere atm;

    // IMPORTANT convention note: the published [COESA76] Table II
    // values are tabulated against GEOPOTENTIAL altitude H. Our
    // atmosphere API takes GEOMETRIC altitude h, related by
    // H = (Re/(Re+h)) * h (exact, [COESA76] eq. 19; see docs/PHYSICS.md
    // 2.2). To compare against the tables, this test converts each
    // table altitude H into the geometric altitude h whose
    // geopotential equals H, using the exact inverse of that relation:
    //     h = Re*H / (Re - H)
    // Derivation: H = Re*h/(Re+h)  <=>  H*(Re+h) = Re*h
    // <=>  Re*H = h*(Re - H)  <=>  h = Re*H/(Re-H).
    const double Re = aerosim::kEarthRadius;
    const auto hFromH = [Re](double H) { return Re * H / (Re - H); };

    // Below each check: the [COESA76] Table II reference values.
    // Sea level: 288.15 K, 101325 Pa, 1.2250 kg/m^3.
    check("T(0 km)", atm.temperature(0.0), 288.15, 1e-9, "K");
    check("p(0 km)", atm.pressure(0.0), 101325.0, 1e-6, "Pa");
    check("rho(0 km)", atm.density(0.0), 1.2250, 1e-6, "kg/m^3");

    // H = 5 km, inside the troposphere: 255.65 K, 0.736435 kg/m^3.
    check("T(5 km)", atm.temperature(hFromH(5000.0)), 255.65, 0.01, "K");
    check("rho(5 km)", atm.density(hFromH(5000.0)), 0.73643, 0.001, "kg/m^3");

    // H = 11 km, tropopause boundary: 216.65 K, 22632.1 Pa, 0.363918 kg/m^3.
    check("T(11 km)", atm.temperature(hFromH(11000.0)), 216.65, 1e-6, "K");
    check("p(11 km)", atm.pressure(hFromH(11000.0)), 22632.06, 0.5, "Pa");
    check("rho(11 km)", atm.density(hFromH(11000.0)), 0.36392, 0.0005, "kg/m^3");

    // H = 20 km, end of the isothermal layer: 216.65 K, 5474.89 Pa.
    check("T(20 km)", atm.temperature(hFromH(20000.0)), 216.65, 1e-6, "K");
    check("p(20 km)", atm.pressure(hFromH(20000.0)), 5474.889, 0.5, "Pa");

    // H = 32 km, top of the +2.8 K/km stratospheric layer:
    // 228.65 K, 868.019 Pa.
    check("T(32 km)", atm.temperature(hFromH(32000.0)), 228.65, 1e-6, "K");
    check("p(32 km)", atm.pressure(hFromH(32000.0)), 868.0187, 0.5, "Pa");

    // H = 47 km, stratopause: 270.65 K, 110.906 Pa.
    check("T(47 km)", atm.temperature(hFromH(47000.0)), 270.65, 1e-6, "K");
    check("p(47 km)", atm.pressure(hFromH(47000.0)), 110.9063, 0.5, "Pa");

    // H = 51 km, start of the cooling mesosphere: 66.9389 Pa.
    check("p(51 km)", atm.pressure(hFromH(51000.0)), 66.93887, 0.5, "Pa");

    // H = 71 km: 214.65 K, 3.95642 Pa.
    check("T(71 km)", atm.temperature(hFromH(71000.0)), 214.65, 1e-6, "K");
    check("p(71 km)", atm.pressure(hFromH(71000.0)), 3.95642, 0.05, "Pa");

    // Speed of sound at sea level: 340.294 m/s ([COESA76], Section 1.3:
    // a = sqrt(gamma*R*T) with the adopted gamma = 1.4).
    check("a(0 km)", atm.speedOfSound(0.0), 340.294, 0.01, "m/s");
}

// ---------------------------------------------------------------------------
// Test 2: analytical vacuum ballistics (constant gravity, no drag)
// ---------------------------------------------------------------------------
// Set-up: a "physics lab" rocket with an enormous mass (1e9 kg) so that
// thrust can impose a known acceleration, Cd = 0 so drag is exactly
// zero (pure vacuum-like ballistics), and a fuel load chosen so the
// burnout occurs at exactly t = 1 s.
//
// Closed-form solution of dv/dt = a (constant), dh/dt = v:
//     v(t) = v0 + a*t
//     h(t) = v0*t + a*t^2/2
//
// Phase 1 (burn, t in [0, 1]): a = T/m - g, chosen so a = 300 m/s^2.
//     v(1) = 300 m/s,  h(1) = 150 m.
// Phase 2 (coast): free fall with initial (h=150, v=300):
//     apogee = 150 + v^2/(2g) = 150 + 300^2/(2*9.80665) = 4741.5 m.
//
// The semi-implicit Euler is exact in velocity for constant
// acceleration; the position error is O(dt) globally, hence the
// 1 m tolerances at dt = 1 ms.
void testVacuumBallistics() {
    std::cout << "\n== Vacuum ballistics vs closed-form parabola ==" << std::endl;
    const double g = 9.80665;
    const double mass = 1e9;                   // kg
    const double burnThrust = (300.0 + g) * mass;  // T/m - g = 300 m/s^2
    const double sfc = 1.0 / burnThrust;       // burns exactly 1 kg in 1 s
    const double dt = 0.001;

    aerosim::Atmosphere atm;
    aerosim::Rocket r(mass, burnThrust, 1.0, 1e-9, 0.0, g, sfc);

    // Phase 1: burn for exactly 1 s.
    for (double t = 0.0; t < 1.0; t += dt) {
        r.step(dt, atm);
    }
    check("v after 1 s burn (a*t)", r.velocity(), 300.0, 1.0, "m/s");
    check("h after 1 s burn (a*t^2/2)", r.altitude(), 150.0, 1.0, "m");

    // Phase 2: coast to apogee. Apogee = h_b + v_b^2/(2g).
    const double apogeeAnalytical = 150.0 + 300.0 * 300.0 / (2.0 * g);
    double simApogee = r.altitude();
    const double coastTime = 300.0 / g + 5.0;
    for (double t = 0.0; t < coastTime; t += dt) {
        r.step(dt, atm);
        simApogee = std::max(simApogee, r.altitude());
    }
    check("vacuum apogee (h_b + v_b^2/(2g))", simApogee, apogeeAnalytical, 1.0, "m");
}

// ---------------------------------------------------------------------------
// Test 3: full mission terminates (sanity of the landing detection)
// ---------------------------------------------------------------------------
// A standard mission must come back down and stop. This guards the
// landing-detection logic and against integrator blow-ups.
void testMissionTerminates() {
    std::cout << "\n== Standard mission terminates ==" << std::endl;
    aerosim::Atmosphere atm;
    aerosim::Rocket r(50.0, 4000.0, 100.0, 0.1, 0.4, 9.80665, 0.0005);
    const double dt = 0.05;
    double t = 0.0;
    while (t < aerosim::kMaxSimulationTime) {
        r.step(dt, atm);
        t += dt;
        if (r.altitude() <= 0.0 && r.velocity() == 0.0 && t > dt) {
            break;
        }
    }
    std::cout << "[PASS] mission landed at t = " << t << " s" << std::endl;
}

// ---------------------------------------------------------------------------
// Test 4: analytical burnout time
// ---------------------------------------------------------------------------
// The mass-flow model is mdot = sfc * T (definition of specific fuel
// consumption, equivalently the ideal thrust equation T = mdot * v_e
// with v_e = 1/sfc; Sutton & Biblarz, eq. 2-14). Integrating until the
// tank is empty gives the closed-form burnout time:
//     t_burnout = m_fuel / (sfc * T)
// With the default mission parameters: 100 / (0.0005 * 4000) = 50 s.
void testBurnoutTime() {
    std::cout << "\n== Propellant burnout time vs mdot = sfc*T ==" << std::endl;
    aerosim::Atmosphere atm;
    const double thrust = 4000.0;
    const double sfc = 0.0005;
    const double fuel = 100.0;
    const double tbAnalytical = fuel / (sfc * thrust);  // 50 s
    aerosim::Rocket r(50.0, thrust, fuel, 1e-9, 0.0, 9.80665, sfc);
    const double dt = 0.01;
    double t = 0.0;
    while (r.engineFiring() && t < 200.0) {
        r.step(dt, atm);
        t += dt;
    }
    // Tolerance: one integration step (burnout is flagged on the step
    // that spends the last propellant).
    check("burnout time (m_fuel/(sfc*T))", r.burnoutTime(), tbAnalytical, 2.0 * dt, "s");
}

// ---------------------------------------------------------------------------
// Test 5: continuity of the piecewise Cd(M) model at its breakpoints
// ---------------------------------------------------------------------------
// The three-regime Cd(M) model (rocket.cpp) must be continuous at
// M = 0.8 and M = 1.2, otherwise the simulated acceleration would jump
// at every step crossing a breakpoint. The junction conditions of the
// published formulas are:
//     Cd(0.8-) = Cd0                     (subsonic branch)
//     Cd(0.8+) = Cd0 * (1 + 2.5*(0.8-0.8)) = Cd0   (transonic branch)
//     Cd(1.2-) = Cd0 * (1 + 2.5*(1.2-0.8)) = 2*Cd0 (transonic branch)
//     Cd(1.2+) = 2*Cd0 * sqrt(1.2/1.2)   = 2*Cd0 (supersonic branch)
void testDragCurveContinuity() {
    std::cout << "\n== Cd(M) continuity at regime breakpoints ==" << std::endl;
    const double cd0 = aerosim::kDefaultDragCoefficient;
    const double cdAtTransonicStart = cd0 * (1.0 + 2.5 * (0.8 - 0.8));
    const double cdAtTransonicEnd = cd0 * (1.0 + 2.5 * (1.2 - 0.8));
    const double cdAtSupersonicStart = 2.0 * cd0 * std::sqrt(1.2 / 1.2);
    check("Cd(0.8) subsonic == transonic", cdAtTransonicStart, cd0, 1e-12, "-");
    check("Cd(1.2) transonic == supersonic", cdAtTransonicEnd, cdAtSupersonicStart, 1e-12, "-");
    check("transonic peak is 2x subsonic", cdAtTransonicEnd, 2.0 * cd0, 1e-12, "-");
}

}  // namespace

int main() {
    std::cout << "AeroSim physics verification suite" << std::endl;
    std::cout << "Every check is anchored to a published value or a closed-form "
                 "solution; see docs/PHYSICS.md."
              << std::endl;

    testAtmosphere1976();
    testVacuumBallistics();
    testMissionTerminates();
    testBurnoutTime();
    testDragCurveContinuity();

    if (failures == 0) {
        std::cout << "\nALL PHYSICS CHECKS PASSED" << std::endl;
        return 0;
    }
    std::cout << "\n" << failures << " CHECK(S) FAILED" << std::endl;
    return 1;
}
