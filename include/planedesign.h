#ifndef PLANEDESIGN_H
#define PLANEDESIGN_H

#include <string>
#include <vector>
#include "naca.h"

/**
 * @file planedesign.h
 * @brief Conceptual airplane design tool based on Raymer, "Aircraft Design:
 * A Conceptual Approach" (AIAA Education Series).
 *
 * Flow: requirements -> initial weight sizing (ch. 3) -> drag buildup (ch. 12)
 * -> constraint analysis T/W vs W/S (ch. 5 & 18) -> lift/thrust/power sizing.
 */

// --- Aircraft category for empty-weight fraction We/W0 = A * W0^c (Raymer Table 3.1, W0 in kg)
enum class AcCategory {
    SailplaneUnpowered, SailplanePowered,
    HomebuiltMetalWood, HomebuiltComposite,
    GeneralAviationSingle, GeneralAviationTwin,
    Agricultural, TwinTurboprop, FlyingBoat,
    JetTrainer, JetFighter, MilitaryCargo, JetTransport,
    UavRecce, UavHighAltitude, UavSmall
};

struct CategoryCoeff { double A; double c; const char* name; };
CategoryCoeff categoryCoeff(AcCategory cat);

// --- Design requirements ("guesses" from the user)
struct Requirements {
    // Mission
    double payloadKg = 180.0;    // payload (kg)
    double crewKg = 80.0;         // crew incl. equipment (kg)
    double rangeKm = 1000.0;     // cruise range (km)
    double cruiseSpeedMs = 70.0; // cruise true airspeed (m/s)
    double cruiseAltM = 2500.0;  // cruise altitude (m)
    double enduranceH = 0.0;     // loiter time (h), 0 = none
    double reserveFuelFrac = 0.06; // reserve + trapped fuel fraction

    // Performance constraints
    double stallSpeedMs = 22.0;  // max stall speed (m/s)
    double takeoffDistM = 400.0; // takeoff ground+obstacle distance (m)
    double obstacleHeightM = 15.0; // obstacle height (m, 15.2 m = 50 ft)
    double climbRateMs = 5.0;    // min rate of climb at sea level (m/s)
    double climbSpeedMs = 40.0;  // speed at which ROC is specified (m/s)
    double maxSpeedMs = 90.0;    // max level speed at sea level (m/s)
    double loadFactor = 0.0;     // sustained-turn design load factor, 0 = skip

    // Propulsion
    bool propDriven = true;      // propeller (piston/turboprop) vs jet
    double propEff = 0.80;       // propeller efficiency
    double sfcKgPerkWh = 0.20;   // prop: kg/kWh (C_bhp ~ 0.33 lb/hp/h) | jet: TSFC kg/(N.h)
    double jetTsfc = 0.55;       // jet TSFC (kg per N per hour), used when !propDriven

    // Geometry guesses
    double aspectRatio = 8.0;
    double taper = 0.5;
    double sweepDeg = 0.0;
    std::string airfoilName = "2412";
    double wettedAreaRatio = 4.0; // Swet/Sref (Raymer ch.12 trends: ~4 wing+ fuselage+tails)
    double clmaxClean = 1.5;     // will be refined from airfoil if >0
    double clmaxFlapDelta = 0.6; // CLmax increment from takeoff flaps
    double muRolling = 0.04;     // ground rolling friction
};

// --- Derived design result
struct DesignResult {
    double w0Kg = 0;        // max takeoff weight
    double weKg = 0;        // empty weight
    double wfKg = 0;        // fuel weight
    double wingAreaM2 = 0;  // wing reference area
    double spanM = 0;
    double clmax = 0;       // actual 3D CLmax (flapped for landing/takeoff analysis)
    double clmaxClean = 0;
    double cd0 = 0;
    double oswald = 0;
    double kInduced = 0;
    double ldMax = 0;
    double twReq = 0;       // required thrust-to-weight at design point
    double pwReq = 0;       // required power-to-weight (W/kg) if prop
    double thrustN = 0;
    double powerW = 0;
    double stallSpeedMs = 0;
    double cruiseLD = 0;
    int iterations = 0;
};

// One point of the constraint lines (matching chart)
struct ConstraintPoint {
    double ws;      // wing loading (N/m^2)
    double twJet;   // required T/W (jet convention, = thrust available/weight)
    double pwProp;  // required P/W (W/kg) if prop-driven
    const char* constraint;
};

class PlaneDesign {
public:
    explicit PlaneDesign(const Requirements& req);

    // Full synthesis: weight iteration + constraint sizing.
    DesignResult synthesize();

    // Constraint analysis: required T/W (or P/W) vs wing loading for each
    // active constraint, evaluated in the range [wsMin, wsMax].
    std::vector<ConstraintPoint> constraintCurve(double wsMin, double wsMax, int n) const;

    // Atmosphere (ISA troposphere)
    static double densityAt(double altM);
    static double soundSpeedAt(double altM);

    const NacaAirfoil& airfoil() const { return foil_; }
    const Requirements& req() const { return req_; }

private:
    Requirements req_;
    NacaAirfoil foil_;

    double emptyWeightFraction(double w0Kg) const;
    double missionFuelFraction(double w0Kg) const;
    double dragPolar(double wsN, double& cd0Out, double& kOut, double& eOut) const;
    double clmax3D() const;
};

#endif
