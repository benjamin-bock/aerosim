#include <cmath>
#include <algorithm>
#include "../include/planedesign.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double RHO0 = 1.225;
static const double G = 9.81;

CategoryCoeff categoryCoeff(AcCategory cat) {
    switch (cat) {
        case AcCategory::SailplaneUnpowered:   return {0.83, -0.05, "Sailplane (unpowered)"};
        case AcCategory::SailplanePowered:     return {0.88, -0.05, "Sailplane (powered)"};
        case AcCategory::HomebuiltMetalWood:   return {1.11, -0.09, "Homebuilt metal/wood"};
        case AcCategory::HomebuiltComposite:   return {1.07, -0.09, "Homebuilt composite"};
        case AcCategory::GeneralAviationSingle: return {2.05, -0.18, "GA single engine"};
        case AcCategory::GeneralAviationTwin:  return {1.40, -0.10, "GA twin engine"};
        case AcCategory::Agricultural:         return {0.72, -0.03, "Agricultural"};
        case AcCategory::TwinTurboprop:        return {0.92, -0.05, "Twin turboprop"};
        case AcCategory::FlyingBoat:          return {1.05, -0.05, "Flying boat"};
        case AcCategory::JetTrainer:           return {1.47, -0.10, "Jet trainer"};
        case AcCategory::JetFighter:           return {2.11, -0.13, "Jet fighter"};
        case AcCategory::MilitaryCargo:        return {0.88, -0.07, "Military cargo"};
        case AcCategory::JetTransport:         return {0.97, -0.06, "Jet transport"};
        case AcCategory::UavRecce:             return {1.53, -0.16, "UAV recce"};
        case AcCategory::UavHighAltitude:      return {2.48, -0.18, "UAV high altitude"};
        case AcCategory::UavSmall:             return {0.86, -0.06, "UAV small"};
    }
    return {2.05, -0.18, "GA single engine"};
}

double PlaneDesign::densityAt(double altM) {
    if (altM < 11000.0)
        return RHO0 * std::pow(1.0 - 2.25577e-5 * altM, 4.2559);
    return 0.3639 * std::exp(-(altM - 11000.0) / 6341.0);
}

double PlaneDesign::soundSpeedAt(double altM) {
    if (altM < 11000.0) {
        double t = 288.15 - 6.5e-3 * altM;
        return std::sqrt(1.4 * 287.05 * t);
    }
    return 295.07;
}

PlaneDesign::PlaneDesign(const Requirements& req) : req_(req), foil_(req.airfoilName) {}

double PlaneDesign::clmax3D() const {
    double ar = req_.aspectRatio;
    double sweepRad = req_.sweepDeg * M_PI / 180.0;
    double cl2d = req_.clmaxClean > 0.0 ? req_.clmaxClean : foil_.clMax();
    cl2d *= std::cos(sweepRad);
    double factor = std::max(0.75, std::min(0.95, 0.95 - 0.02 * std::max(0.0, 10.0 - ar)));
    return cl2d * factor;
}

double PlaneDesign::dragPolar(double wsN, double& cd0Out, double& kOut, double& eOut) const {
    (void)wsN;
    double cfe = 0.0055;               // equivalent skin friction, light prop aircraft (Raymer 12.3)
    cd0Out = cfe * req_.wettedAreaRatio;
    double ar = req_.aspectRatio;
    eOut = std::max(0.60, 1.78 * (1.0 - 0.045 * std::pow(ar, 0.68)) - 0.64); // Raymer 12.48
    kOut = 1.0 / (M_PI * ar * eOut);
    return cd0Out;
}

double PlaneDesign::emptyWeightFraction(double w0Kg) const {
    AcCategory cat = req_.propDriven ? AcCategory::GeneralAviationSingle : AcCategory::JetTransport;
    CategoryCoeff cc = categoryCoeff(cat);
    return cc.A * std::pow(std::max(w0Kg, 1.0), cc.c);
}

double PlaneDesign::missionFuelFraction(double w0Kg) const {
    (void)w0Kg;
    double cd0, k, e;
    dragPolar(600.0, cd0, k, e);
    double ldMax = 0.5 * std::sqrt(1.0 / (cd0 * k));
    double ldCruise = req_.propDriven ? ldMax : 0.866 * ldMax;
    double ldLoiter = req_.propDriven ? 0.866 * ldMax : ldMax;
    double rangeM = req_.rangeKm * 1000.0;

    double wf = 0.97 * 0.985 * 0.992 * 0.995; // taxi/climb/descent/land (Raymer Table 3.2)
    if (req_.propDriven) {
        // Prop Breguet (SI, W in kgf): Wi/Wi-1 = exp(-R * c * g / (eta_p * L/D))
        // c: kg/kWh -> kg/(W.s)
        double c = req_.sfcKgPerkWh / 3.6e6;
        wf *= std::exp(-rangeM * G * c / (req_.propEff * ldCruise));
        if (req_.enduranceH > 0.0)
            wf *= std::exp(-req_.enduranceH * 3600.0 * G * c / (req_.propEff * ldLoiter));
    } else {
        // Jet Breguet: Wi/Wi-1 = exp(-R * cj / (V * L/D)); loiter: exp(-E * cj / L/D)
        double cj = req_.jetTsfc / 3600.0; // 1/s
        wf *= std::exp(-rangeM * cj / (req_.cruiseSpeedMs * ldCruise));
        if (req_.enduranceH > 0.0)
            wf *= std::exp(-req_.enduranceH * 3600.0 * cj / ldLoiter);
    }
    return (1.0 + req_.reserveFuelFrac) * (1.0 - wf);
}

DesignResult PlaneDesign::synthesize() {
    DesignResult r;
    AcCategory cat = AcCategory::GeneralAviationSingle;
    if (!req_.propDriven) cat = AcCategory::JetTransport;
    CategoryCoeff cc = categoryCoeff(cat);

    double w0 = 4.0 * (req_.payloadKg + req_.crewKg);
    double weFrac = 0.55;
    int it = 0;
    for (it = 0; it < 100; ++it) {
        double wfFrac = missionFuelFraction(w0);
        if (wfFrac >= 0.75) wfFrac = 0.75;
        double denom = 1.0 - weFrac - wfFrac;
        if (denom <= 0.05) { w0 *= 1.1; continue; }
        double w0New = (req_.crewKg + req_.payloadKg) / denom;
        weFrac = cc.A * std::pow(std::max(w0New, 1.0), cc.c); // Raymer Table 3.1 (kg)
        if (std::fabs(w0New - w0) < 0.01 * w0) { w0 = w0New; break; }
        w0 = w0New;
    }
    r.iterations = it + 1;
    r.w0Kg = w0;
    r.weKg = weFrac * w0;
    r.wfKg = missionFuelFraction(w0) * w0;

    double clmaxTko = clmax3D() + req_.clmaxFlapDelta;
    double clClean = clmax3D();
    r.clmaxClean = clClean;
    r.clmax = clmaxTko;

    // Wing loading from stall speed (governing) and cruise CL reasonableness
    double wsStall = 0.5 * RHO0 * req_.stallSpeedMs * req_.stallSpeedMs * clmaxTko;
    double ws = std::max(50.0, wsStall);
    r.wingAreaM2 = w0 * G / ws;
    r.spanM = std::sqrt(req_.aspectRatio * r.wingAreaM2);

    double cd0, k, e;
    dragPolar(ws, cd0, k, e);
    r.cd0 = cd0;
    r.oswald = e;
    r.kInduced = k;
    r.ldMax = 0.5 * std::sqrt(1.0 / (cd0 * k));

    double qCr = 0.5 * densityAt(req_.cruiseAltM) * req_.cruiseSpeedMs * req_.cruiseSpeedMs;
    double clCruise = (w0 * G) / (qCr * r.wingAreaM2);
    r.cruiseLD = clCruise / (cd0 + k * clCruise * clCruise);

    double weight = w0 * G;
    double S = r.wingAreaM2;

    // Climb requirement at sea level
    double thrustNeed;
    {
        double v = req_.climbSpeedMs;
        double q = 0.5 * RHO0 * v * v;
        double cl = weight / (q * S);
        double d = q * S * (cd0 + k * cl * cl);
        double gamma = std::asin(std::min(1.0, req_.climbRateMs / v));
        thrustNeed = weight * std::sin(gamma) + d;
    }
    // Max level speed, sea level
    {
        double v = req_.maxSpeedMs;
        double q = 0.5 * RHO0 * v * v;
        double cl = weight / (q * S);
        double d = q * S * (cd0 + k * cl * cl);
        thrustNeed = std::max(thrustNeed, d);
    }
    // Cruise at altitude
    {
        double cl = clCruise;
        double d = qCr * S * (cd0 + k * cl * cl);
        thrustNeed = std::max(thrustNeed, d);
    }
    // Turn load factor
    if (req_.loadFactor > 1.0) {
        double v = req_.maxSpeedMs * 0.7;
        double q = 0.5 * RHO0 * v * v;
        double cl = req_.loadFactor * weight / (q * S);
        double d = q * S * (cd0 + k * cl * cl);
        thrustNeed = std::max(thrustNeed, req_.loadFactor * d);
    }
    // Takeoff + obstacle: energy method
    double vTO = 1.1 * std::sqrt(2.0 * ws / (RHO0 * clmaxTko));
    {
        double qLO = 0.5 * RHO0 * vTO * vTO;
        double clTO = clmaxTko * 0.9;
        double d = qLO * S * (cd0 + k * clTO * clTO);
        double gammaObs = std::atan2(req_.obstacleHeightM, std::max(0.55 * req_.takeoffDistM, 1.0));
        double tObs = weight * std::sin(gammaObs) + d;
        // Ground roll: mean accel over roll to vTO; prop thrust ~ P*eta/v_mean
        double sGround = std::max(0.55 * req_.takeoffDistM, 30.0); // part of field before obstacle arc
        double liftMean = 0.5 * weight; // lift builds up; conservative
        double fric = req_.muRolling * (weight - liftMean);
        double dMean = 0.5 * qLO * S * (cd0 + k * clTO * clTO);
        // Required thrust at vMean: T = [0.5*m*vTO^2/s + fric + dMean]*... via T(vMean)
        double m = w0;
        double accel = (0.5 * m * vTO * vTO) / (sGround * m); // mean accel (m/s^2)
        double tRoll = m * (accel * 1.0) + fric + dMean;
        thrustNeed = std::max(thrustNeed, std::max(tObs, tRoll));
    }

    r.twReq = thrustNeed / weight;
    r.thrustN = thrustNeed;
    if (req_.propDriven) {
        // Convert the governing thrust requirement to shaft power at its own speed
        double vRef = req_.climbSpeedMs;
        // climb power (governing typical for prop): P = T_climb * V_climb / eta
        double q = 0.5 * RHO0 * vRef * vRef;
        double cl = weight / (q * S);
        double d = q * S * (cd0 + k * cl * cl);
        double gamma = std::asin(std::min(1.0, req_.climbRateMs / vRef));
        double tClimb = weight * std::sin(gamma) + d;
        double pClimb = tClimb * vRef / req_.propEff;
        // takeoff power: T(vMean) -> P = T * vMean / eta
        double vMean = 0.7 * vTO;
        double qLO = 0.5 * RHO0 * vTO * vTO;
        double clTO = clmaxTko * 0.9;
        double dTO = qLO * S * (cd0 + k * clTO * clTO);
        double gammaObs = std::atan2(req_.obstacleHeightM, std::max(0.55 * req_.takeoffDistM, 1.0));
        double tObs = weight * std::sin(gammaObs) + dTO;
        double sGround = std::max(0.55 * req_.takeoffDistM, 30.0);
        double liftMean = 0.5 * weight;
        double fric = req_.muRolling * (weight - liftMean);
        double dMean = 0.5 * qLO * S * (cd0 + k * clTO * clTO);
        double accel = (0.5 * w0 * vTO * vTO) / (sGround * w0);
        double tRoll = w0 * accel + fric + dMean;
        double pTakeoff = std::max(tObs * vTO, tRoll * vMean) / req_.propEff;
        // max-speed power: P = D * V / eta (thrust-limited prop approximation)
        double pMax = 0.0;
        {
            double v = req_.maxSpeedMs;
            double q = 0.5 * RHO0 * v * v;
            double cl = weight / (q * S);
            double d = q * S * (cd0 + k * cl * cl);
            pMax = d * v / req_.propEff;
        }
        r.powerW = std::max(std::max(pClimb, pTakeoff), pMax);
        r.pwReq = r.powerW / w0;
    } else {
        r.powerW = 0.0;
        r.pwReq = 0.0;
    }
    r.stallSpeedMs = std::sqrt(2.0 * ws / (RHO0 * clmaxTko));
    return r;
}

std::vector<ConstraintPoint> PlaneDesign::constraintCurve(double wsMin, double wsMax, int n) const {
    std::vector<ConstraintPoint> pts;
    double cd0, k, e;
    dragPolar(600.0, cd0, k, e);
    double clmaxTko = clmax3D() + req_.clmaxFlapDelta;
    double w0 = 1000.0;
    double weight = w0 * G;

    for (int i = 0; i <= n; ++i) {
        double ws = wsMin + (wsMax - wsMin) * i / n;
        double S = weight / ws;

        // Stall: W/S <= 0.5*rho*Vstall^2*CLmax
        double wsStallMax = 0.5 * RHO0 * req_.stallSpeedMs * req_.stallSpeedMs * clmaxTko;

        // Takeoff (energy method, prop-aware)
        double vTO = 1.1 * std::sqrt(2.0 * ws / (RHO0 * clmaxTko));
        double qLO = 0.5 * RHO0 * vTO * vTO;
        double clTO = clmaxTko * 0.9;
        double dTO = qLO * S * (cd0 + k * clTO * clTO);
        double sGround = std::max(0.55 * req_.takeoffDistM, 30.0);
        double gammaObs = std::atan2(req_.obstacleHeightM, std::max(0.55 * req_.takeoffDistM, 1.0));
        double tObs = weight * std::sin(gammaObs) + dTO;
        double accel = (0.5 * vTO * vTO) / sGround;
        double vMean = 0.7 * vTO;
        double liftMean = 0.5 * weight;
        double fric = req_.muRolling * (weight - liftMean);
        double dMean = 0.5 * qLO * S * (cd0 + k * clTO * clTO);
        double tRoll = w0 * accel + fric + dMean;
        double tTakeoff = std::max(tObs, tRoll);

        // Climb
        double v = req_.climbSpeedMs;
        double q = 0.5 * RHO0 * v * v;
        double clC = weight / (q * S);
        double dC = q * S * (cd0 + k * clC * clC);
        double gamma = std::asin(std::min(1.0, req_.climbRateMs / v));
        double tClimb = weight * std::sin(gamma) + dC;

        // Max speed
        double vM = req_.maxSpeedMs;
        double qM = 0.5 * RHO0 * vM * vM;
        double clM = weight / (qM * S);
        double tMax = qM * S * (cd0 + k * clM * clM);

        double twJet = std::max(std::max(tTakeoff, tClimb), tMax) / weight;
        double pwProp = req_.propDriven
            ? std::max(std::max(tTakeoff * vMean, tClimb * v), tMax * vM) / req_.propEff / w0
            : 0.0;

        ConstraintPoint pt;
        pt.ws = ws;
        pt.twJet = twJet;
        pt.pwProp = pwProp;
        pt.constraint = (ws > wsStallMax) ? "stall-violated" : "ok";
        pts.push_back(pt);
    }
    return pts;
}
