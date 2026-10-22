#include <cstdio>
#include <cmath>
#include "../include/synthesis.h"

/**
 * @file calibration.cpp
 * @brief Validation against a real aircraft: Cessna 172 Skyhawk (1981 model,
 * Lycoming O-320). Reference data from the POH / public specs.
 */
int main() {
    // Cessna 172N real data
    const double W0_real = 1111;       // kg MTOW (2450 lb)
    const double S_real = 16.2;        // m2
    const double b_real = 11.0;        // m
    const double power_real = 120;     // kW (160 hp)
    const double vstall_real = 26.0;   // m/s clean ~ 49 kt CAS (50.5 kt)
    const double range_real = 1150;    // km with 53 gal usable, ~ profile dependent
    const double cruise_real = 60;     // m/s (122 ktas)
    const double ceiling_real = 4210;  // m (13,800 ft)

    // Spec the tool to the 172 mission
    DesignSpec spec;
    // Realistic 172 mission: full fuel (144 kg) + 3 pax + pilot ~= 340 kg useful
    // load total; the synthesis adds its own fuel, so payload = useful - fuel.
    spec.payloadKg = 200;             // 3 pax + light baggage
    spec.crewKg = 80;                 // pilot
    spec.rangeKm = 1000;              // ~ range at 75% power with reserves
    spec.cruiseSpeedMs = 60;
    spec.cruiseAltM = 2500;
    spec.stallSpeedMs = 26;
    spec.maxSpeedMs = 68;
    spec.rocMs = 3.8;
    spec.climbSpeedMs = 36;
    spec.takeoffDistM = 330;
    spec.airfoil = "2412";
    spec.aspectRatio = 7.5;
    spec.flap = FlapType::SingleSlot;
    spec.nSeats = 4;
    spec.propEff = 0.80;

    SynthesisResult r = synthesizeDesign(spec);

    auto cmp = [](const char* n, double got, double want, double tol) {
        double err = std::fabs(got - want) / want;
        bool ok = err <= tol;
        std::printf("%-22s got %8.1f  want %8.1f  err %5.1f%%  %s\n",
                    n, got, want, err * 100, ok ? "OK" : "OUT");
        return ok;
    };

    std::printf("=== Cessna 172 calibration ===\n");
    cmp("W0 [kg]", r.w0Kg, W0_real, 0.15);
    cmp("Wing area [m2]", r.wingArea, S_real, 0.15);
    cmp("Span [m]", r.span, b_real, 0.12);
    cmp("Power [kW]", r.powerKW, power_real, 0.30);
    cmp("Stall [m/s]", r.perf.vStallClean, vstall_real, 0.15);
    // POH range includes 45-min reserve; model uses 6% -> compare reserve-free
    cmp("Range [km]", r.perf.rangeKm, range_real * 1.25, 0.20);
    cmp("Cruise L/D [-]", r.aero.ldMax, 11.0, 0.35);
    cmp("Ceiling [m]", r.perf.ceilingM, ceiling_real, 0.35);
    std::printf("CG at %.2f m, static margin %.1f%% MAC (%s)\n",
                r.stab.cgM, r.stab.staticMargin * 100,
                r.stab.cgWithinLimits ? "stable" : "CHECK");
    std::printf("Engine selected: %s\n",
                r.engineIdx >= 0 ? engineDatabase()[r.engineIdx].model : "none");
    return 0;
}
