#pragma once
#include "common.h"

namespace aerosim {

struct BLPoint {
    double x;
    double theta;
    double delta_str;
    double cf;
    double ue;
    bool turbulent;
    bool separated;
};

struct BLTrajectory {
    std::vector<BLPoint> points;
    bool laminar_separation = false;
    double x_laminar_sep = 0.0;
    bool turbulent_separation = false;
    double x_turbulent_sep = 1.0;
};

struct BLEntry {
    double x = 0.0;
    double theta = 0.0;
    double dstar = 0.0;
    double cf = 0.0;
    bool turbulent = false;
    bool separated = false;
};

struct BLSurfaceResult {
    std::vector<BLEntry> entries;
    double x_trans = 1.0;
    double x_sep = 1.0;
    bool separated = false;
};

BLSurfaceResult bl_solve_surface(const SampledFunction& ue, double x_le,
                                 double reynolds, double x_start,
                                 double xtr_forced, bool fully_turbulent);

}  // namespace aerosim
