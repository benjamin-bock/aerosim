#include <cmath>
#include <algorithm>
#include "../include/trade.h"

static const double G = 9.81;
static const double RHO0 = 1.225;

TradeStudyResult carpetStudy(const DesignSpec& baseSpec,
                             const std::vector<double>& arValues,
                             const std::vector<double>& wsValues,
                             double wsStallOverride) {
    TradeStudyResult r;
    r.arValues = arValues;
    r.wsValues = wsValues;

    // synthesizeDesign picks ws from stall; here we need a variant honoring a
    // given W/S, so we replicate the loop with fixed ws by adjusting the
    // stall-speed requirement: Vstall' = sqrt(2*ws/(rho*clmaxTO)).
    for (double ar : arValues) {
        for (double ws : wsValues) {
            DesignSpec spec = baseSpec;
            spec.aspectRatio = ar;
            // First run with default to get clmaxTO estimate
            SynthesisResult probe = synthesizeDesign(spec);
            double clmaxTO = probe.aero.clmaxTakeoff;
            // Set stall speed so that the sizing ws matches requested ws
            double vStall = std::sqrt(2.0 * ws / (RHO0 * clmaxTO));
            spec.stallSpeedMs = vStall;
            SynthesisResult s = synthesizeDesign(spec);
            TradePoint p;
            p.ar = ar;
            p.ws = ws;
            p.w0 = s.w0Kg;
            p.powerKW = s.powerKW;
            p.ldMax = s.aero.ldMax;
            p.converged = s.converged;
            r.grid.push_back(p);
            if (!r.found && s.converged) {
                r.optimum = p;
                r.found = true;
            } else if (s.converged && p.w0 < r.optimum.w0) {
                r.optimum = p;
            }
        }
    }
    (void)wsStallOverride;
    return r;
}
