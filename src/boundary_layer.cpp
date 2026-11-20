#include "boundary_layer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerosim {

namespace {

double thwaites_l(double lambda) {
    if (lambda <= -0.1) return 0.0;
    if (lambda < -0.0681) return (lambda + 0.1) * 0.9 / 0.0319;
    if (lambda <= 0.1) return 0.22 + 1.57 * lambda - 1.8 * lambda * lambda;
    return 0.22 + 1.57 * 0.1 - 1.8 * 0.01;
}

double thwaites_h(double lambda) {
    if (lambda <= -0.1) return 4.0;
    if (lambda < -0.0681) return 4.0 + (lambda + 0.1) * (2.216 - 4.0) / 0.0319;
    if (lambda <= 0.1) return 2.61 - 3.75 * lambda + 24.0 * lambda * lambda;
    return 2.61 - 3.75 * 0.1 + 24.0 * 0.01;
}

double ludwieg_tillmann(double H, double re_theta) {
    double Hc = std::min(std::max(H, 1.1), 4.5);
    double rt = std::max(re_theta, 50.0);
    return 0.246 * std::pow(10.0, -0.678 * Hc) * std::pow(rt, -0.268);
}

double head_H1_of_H(double H) {
    double Hc = std::min(std::max(H, 1.1001), 4.5);
    if (Hc <= 1.6) return 3.3 + 0.8234 * std::pow(Hc - 1.1, -1.287);
    return 3.3 + 1.5501 * std::pow(Hc - 0.6778, -3.064);
}

double head_H_of_H1(double H1) {
    double lo = 1.05, hi = 5.0;
    for (int it = 0; it < 80; ++it) {
        double mid = 0.5 * (lo + hi);
        if (head_H1_of_H(mid) > H1)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}

double head_F(double H1) {
    double h1 = std::max(H1, 3.005);
    return 0.0306 * std::pow(h1 - 3.0, -0.6169);
}

}  // namespace

BLSurfaceResult bl_solve_surface(const SampledFunction& ue, double x_le,
                                 double reynolds, double x_start,
                                 double xtr_forced, bool fully_turbulent) {
    BLSurfaceResult res;
    if (ue.xs.size() < 3) throw std::runtime_error("ue function too small");
    double nu = 1.0 / std::max(reynolds, 1e4);
    int n = static_cast<int>(ue.xs.size());

    bool transitioned = fully_turbulent;
    bool laminar_sep = false;
    bool turb_sep = false;
    double x_trans = 1.0;
    double x_sep = 1.0;
    double theta = 0.0;
    double H = 2.59;
    bool first = true;
    double cf = 0.0;
    int steps_after_trans = 0;

    for (int i = 0; i < n; ++i) {
        double x = ue.xs[i];
        double u = std::max(ue.eval(x), 1e-8);
        if (x < x_start - 1e-12) continue;
        BLEntry e;
        e.x = x;
        if (first) {
            double dx_eff = std::max(x - x_le, 1e-5);
            theta = 0.071 * std::sqrt(nu * dx_eff / u);
            theta = std::max(theta, 1e-7);
            H = 2.59;
            double re_theta = u * theta / nu;
            cf = 2.0 * thwaites_l(0.0) / std::max(re_theta, 1.0);
            first = false;
        } else {
            double dx = x - res.entries.back().x;
            if (dx <= 1e-12) continue;
            double u_prev = std::max(ue.eval(res.entries.back().x), 1e-8);
            double u_mid = std::max(0.5 * (u + u_prev), 1e-8);
            double du = (u - u_prev) / dx;
            if (!transitioned) {
                double th_prev = theta;
                double theta2 = th_prev * th_prev * std::pow(u_prev / u, 5.0) +
                                0.45 * nu * dx * std::pow(u_mid, 4.0) / std::pow(u, 5.0);
                theta = std::sqrt(std::max(theta2, 1e-20));
                double lambda = theta * theta * du / (nu * u_mid);
                double lambda_c = std::min(lambda, 0.1);
                H = thwaites_h(lambda_c);
                double re_theta = u_mid * theta / nu;
                cf = 2.0 * thwaites_l(lambda_c) / std::max(re_theta, 1.0);
                double re_x = u_mid * std::max(x - x_le, 1e-6) / nu;
                double re_theta_michel = 2.9 * std::pow(re_x, 0.4);
                bool force_now = (xtr_forced >= 0.0 && x >= xtr_forced);
                int n_so_far = static_cast<int>(res.entries.size());
                bool bubble_ok = re_theta > 150.0 && n_so_far >= 3;
                if (lambda < -0.09 && bubble_ok) {
                    laminar_sep = true;
                    transitioned = true;
                    x_trans = x;
                    H = 2.8;
                } else if (re_theta > re_theta_michel || force_now) {
                    transitioned = true;
                    x_trans = x;
                    H = 2.4;
                }
            } else {
                double re_theta = u_mid * theta / nu;
                cf = ludwieg_tillmann(H, re_theta);
                double dtheta = (cf / 2.0 - (H + 2.0) * theta * du / u_mid) * dx;
                double theta_new = std::max(theta + dtheta, 1e-9);
                double H1 = head_H1_of_H(H);
                double F = head_F(H1);
                double dH1 = (F * u_mid - theta_new * H1 * du - u_mid * H1 *
                              (cf / 2.0 - (H + 2.0) * theta_new * du / u_mid)) /
                             (u_mid * theta_new);
                dH1 *= dx;
                double H1_new = H1 + dH1;
                H = head_H_of_H1(std::min(std::max(H1_new, 3.0), 39.0));
                H = std::min(std::max(H, 1.2), 4.8);
                theta = theta_new;
                steps_after_trans += 1;
                if (H > 2.7 && du < 0 && re_theta > 300 && steps_after_trans > 8 &&
                    !turb_sep) {
                    turb_sep = true;
                    x_sep = x;
                }
            }
        }
        e.theta = theta;
        e.dstar = H * theta;
        e.cf = std::max(cf, 0.0);
        e.turbulent = transitioned;
        e.separated = turb_sep || (laminar_sep && !transitioned);
        res.entries.push_back(e);
        if (turb_sep) break;
    }
    res.x_trans = x_trans;
    res.x_sep = turb_sep ? x_sep : 1.0;
    res.separated = turb_sep;
    return res;
}

}  // namespace aerosim
