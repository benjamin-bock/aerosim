#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include "../include/atmosphere.h"
#include "../include/naca.h"
#include "../include/geometry.h"
#include "../include/aero.h"
#include "../include/weights.h"
#include "../include/constraints.h"
#include "../include/engines.h"
#include "../include/performance.h"
#include "../include/synthesis.h"

static int failures = 0, tests = 0;

static void check(bool ok, const char* name) {
    ++tests;
    if (!ok) { ++failures; std::printf("FAIL: %s\n", name); }
    else std::printf("ok  : %s\n", name);
}

static void checkNear(double v, double ref, double tol, const char* name) {
    ++tests;
    bool ok = std::fabs(v - ref) <= tol * std::fabs(ref);
    if (!ok) { ++failures; std::printf("FAIL: %s (got %.4f, want %.4f +/- %.1f%%)\n", name, v, ref, tol * 100); }
    else std::printf("ok  : %s\n", name);
}

int main() {
    // --- Atmosphere
    Atmosphere sl = isaAt(0.0);
    checkNear(sl.density, 1.225, 0.01, "ISA SL density = 1.225");
    checkNear(sl.pressure, 101325.0, 0.01, "ISA SL pressure = 101325 Pa");
    checkNear(sl.temperature, 288.15, 0.001, "ISA SL temp = 288.15 K");
    Atmosphere fl110 = isaAt(11000.0);
    checkNear(fl110.temperature, 216.65, 0.001, "ISA 11km temp = 216.65 K");
    checkNear(fl110.pressure, 22632.0, 0.01, "ISA 11km pressure = 22632 Pa");
    checkNear(fl110.density, 0.3639, 0.01, "ISA 11km density = 0.3639");
    check(isaDensity(5000.0) < isaDensity(0.0), "density decreases with altitude");

    // --- NACA airfoils
    NacaAirfoil n2412("2412");
    checkNear(n2412.tc(), 0.12, 0.001, "NACA 2412 t/c = 0.12");
    checkNear(n2412.camberMax(), 0.02, 0.001, "NACA 2412 max camber = 0.02");
    checkNear(n2412.alpha0() * 180 / M_PI, -2.08, 0.05, "NACA 2412 alpha_L0 ~ -2.1 deg");
    check(n2412.clMax() > 1.2 && n2412.clMax() < 1.7, "NACA 2412 CLmax in [1.2,1.7]");
    NacaAirfoil n0012("0012");
    check(std::fabs(n0012.alpha0()) < 1e-6, "NACA 0012 symmetric: alpha0 = 0");
    checkNear(n0012.thicknessAt(0.3), 0.0600, 0.03, "0012 half-thickness at 30% chord ~ 0.06");
    NacaAirfoil n23012("23012");
    checkNear(n23012.tc(), 0.12, 0.001, "NACA 23012 t/c = 0.12");
    NacaAirfoil n63209("63-209");
    checkNear(n63209.tc(), 0.09, 0.001, "NACA 63-209 t/c = 0.09");
    NacaAirfoil n64215("64(2)215");
    checkNear(n64215.tc(), 0.15, 0.001, "NACA 64(2)215 t/c = 0.15");

    // --- Geometry
    NacaAirfoil foil("2412");
    LayoutInputs li; li.nSeats = 2;
    Geometry g = layoutAircraft(15.0, 8.0, 0.5, 0.0, foil, li);
    checkNear(g.span, std::sqrt(120.0), 0.001, "span = sqrt(AR*S)");
    check(g.mac > 1.0 && g.mac < 2.0, "MAC in plausible range for S=15, AR=8");
    check(g.swetTotal > g.wingArea, "wetted area > wing area");
    check(g.htArea > 0.3 && g.htArea < 0.5 * g.wingArea, "htail area plausible");

    // --- Aero
    AeroResult a = aeroAnalysis(g, foil, 70.0, 2500.0, FlapType::Plain, 0.25, 0.6);
    check(a.cd0 > 0.015 && a.cd0 < 0.045, "CD0 in plausible GA range");
    check(a.oswald > 0.6 && a.oswald < 0.95, "Oswald in range");
    check(a.ldMax > 8.0 && a.ldMax < 25.0, "(L/D)max plausible");
    check(a.clmaxTakeoff > a.clmaxClean, "flaps increase CLmax");
    AeroResult aNoFlap = aeroAnalysis(g, foil, 70.0, 2500.0, FlapType::None);
    check(a.clmaxTakeoff > aNoFlap.clmaxClean, "takeoff CLmax > clean");
    check(a.clAlpha3D > 3.0 && a.clAlpha3D < 7.0, "3D lift slope plausible (per rad)");

    // --- Weights: buildup fraction close to Raymer Table 3.1 regression
    WeightInputs wi;
    wi.w0Kg = 900; wi.engineMassKg = 118; wi.fuelKg = 100; wi.fuelInWingKg = 50;
    wi.cruiseLD = 12; wi.cruiseDynamicPressure = 3000;
    WeightResult w = componentWeights(g, wi);
    checkNear(w.buildupFrac, w.table31Frac, 0.15, "We/W0 buildup within 15% of Table 3.1");
    check(w.wingKg > 30 && w.wingKg < 250, "wing weight plausible");

    // --- Constraints: monotonicity checks
    ConstraintSet cs;
    cs.vStall = 22; cs.vClimb = 40; cs.roc = 5; cs.vMax = 90;
    cs.vCruise = 70; cs.altCruise = 2500; cs.takeoffDist = 400;
    cs.clmaxTO = 1.6; cs.propDriven = true;
    ConstraintAnalysis ca(a, cs);
    auto lines = ca.compute(100.0, 2000.0, 20);
    check(lines.size() >= 4, "constraint lines computed");
    double tw1, pw1; const char* lbl;
    bool ok1 = ca.requirement(300.0, tw1, pw1, lbl);
    check(ok1, "requirement() feasible at low W/S");
    double tw2, pw2; const char* lbl2;
    bool ok2 = ca.requirement(2000.0, tw2, pw2, lbl2);
    check(!ok2, "requirement() infeasible above stall cap");
    double tw3, pw3; const char* lbl3;
    bool ok3 = ca.requirement(400.0, tw3, pw3, lbl3);
    check(ok3 && pw3 > 0, "prop P/W requirement positive");

    // --- Engines
    const auto& db = engineDatabase();
    check(db.size() >= 10, "engine database populated");
    int idx = selectEngine(80.0);
    check(idx >= 0 && db[idx].powerKW >= 80.0, "engine selection meets power");
    check(std::strcmp(db[idx].model, "Rotax 914 UL") == 0, "80kW -> Rotax 914");

    // --- Performance (sanity on a converged-ish design)
    PerformanceInputs pi;
    pi.w0Kg = 900; pi.fuelKg = 100; pi.powerW = 120000; pi.sfcKgPerkWh = 0.30;
    pi.cruiseAltM = 2500; pi.cruiseSpeedMs = 70; pi.clmaxTO = 1.6;
    PerformanceResult pr = analyzePerformance(a, g, pi);
    check(pr.vStallClean > 15 && pr.vStallClean < 40, "stall speed plausible");
    check(pr.rangeKm > 300, "range > 300 km with 100 kg fuel");
    check(pr.rocSLms > 2.0, "ROC at SL > 2 m/s");
    check(pr.takeoffDistM > 50 && pr.takeoffDistM < 1500, "takeoff distance plausible");
    check(pr.ceilingM > 2000, "service ceiling > 2000 m");

    // --- Synthesis: converges and closes the weight loop
    DesignSpec spec;
    spec.payloadKg = 180; spec.rangeKm = 1000;
    SynthesisResult sr = synthesizeDesign(spec);
    check(sr.converged, "synthesis converges");
    checkNear(sr.w0Kg, sr.weights.totalEmptyKg + spec.crewKg + spec.payloadKg + sr.wfKg, 0.02,
              "weight loop closes: W0 = We + crew + payload + fuel");
    check(sr.wfKg / sr.w0Kg < 0.25, "fuel fraction < 25% for 1000 km mission");
    check(sr.perf.rangeKm >= spec.rangeKm, "achieved range >= requirement");

    std::printf("\n%d/%d tests passed\n", tests - failures, tests);
    return failures == 0 ? 0 : 1;
}
