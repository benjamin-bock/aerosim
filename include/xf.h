#pragma once
#include "common.h"
#include "profile.h"

namespace aerosim {

struct XFOptions {
    int n_panels = 160;
    int max_iter = 60;
    double relaxation = 0.3;
    double tol = 1e-4;
    bool fully_turbulent = false;
    bool force_transition_top = -1.0;
    double xtr_top = -1.0;
    double xtr_bot = -1.0;
    bool incompressible_bl = true;
};

struct XFDistributions {
    std::vector<double> x;
    std::vector<double> cp;
    std::vector<double> ue;
    std::vector<double> cf;
    std::vector<double> theta;
    std::vector<double> dstar;
    std::vector<int> side;
    double xtr_top = 1.0, xtr_bot = 1.0;
    double xsep_top = 1.0, xsep_bot = 1.0;
};

CaseResult solve_case(const Profile& prof, double alpha_deg, double reynolds,
                      double mach, const XFOptions& opt,
                      XFDistributions* dist = nullptr);

Polar solve_polar(const Profile& prof, double reynolds, double mach,
                  double alpha_min, double alpha_max, double alpha_step,
                  const XFOptions& opt, bool stop_on_stall = true,
                  std::vector<CaseResult>* raw = nullptr);

struct OptimizeResult {
    Polar best_polar;
    double cl_max = 0.0;
    double cd_at_clmax = 0.0;
    double ld_max = 0.0;
    double cl_at_ldmax = 0.0;
    double alpha_clmax = 0.0;
    double alpha_ldmax = 0.0;
    double stall_margin = 0.0;
};

OptimizeResult optimize_profile(Profile& prof, const XFOptions& opt,
                               double reynolds, double mach,
                               double alpha_min, double alpha_max,
                               double alpha_step);

}  // namespace aerosim
