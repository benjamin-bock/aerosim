#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace aerosim {

constexpr double kPi = 3.14159265358979323846;

struct PolarPoint {
    double alpha_deg;
    double cl;
    double cd;
    double cm;
    double xtr_top;
    double xtr_bot;
};

struct Polar {
    std::vector<PolarPoint> points;
};

struct DistributionsPoint {
    double x;
    double y;
    double v;
    double cp;
    double cf;
    double ue;
};

struct TransitionResult {
    bool top_transitioned = false;
    double xtr_top = 1.0;
    bool bot_transitioned = false;
    double xtr_bot = 1.0;
    bool top_separated = false;
    double xsep_top = 1.0;
    bool bot_separated = false;
    double xsep_bot = 1.0;
};

struct CaseResult {
    double alpha_deg;
    double reynolds;
    double mach;
    double cl = 0.0;
    double cd = 0.0;
    double cm = 0.0;
    double cd_friction = 0.0;
    double cd_pressure = 0.0;
    double cd_profil = 0.0;
    bool converged = false;
    TransitionResult trans;
};

double deg_to_rad(double deg);
double rad_to_deg(double rad);

void spline_coefs(const std::vector<double>& x, const std::vector<double>& y,
                  std::vector<double>& a, std::vector<double>& b,
                  std::vector<double>& c, std::vector<double>& d);
double spline_eval(const std::vector<double>& x, const std::vector<double>& a,
                   const std::vector<double>& b, const std::vector<double>& c,
                   const std::vector<double>& d, double xv);

struct SampledFunction {
    std::vector<double> xs;
    std::vector<double> ys;
    double a0;
    std::vector<double> sa, sb, sc, sd;

    void build(bool close_kutta);
    double eval(double x) const;
    double deriv(double x) const;
    double integrate(double x0, double x1) const;
};

std::vector<double> lin_space(double x0, double x1, int n);

}  // namespace aerosim
