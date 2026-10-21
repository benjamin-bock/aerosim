#include "xf.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "boundary_layer.h"

namespace aerosim {

namespace {

struct HSPanel {
    double x1, y1, x2, y2;
    double mx, my;
    double ds;
    double tx, ty;
    double nx, ny;
    bool upper;
    double theta;
};

std::vector<HSPanel> build_hs_panels(const Profile& prof, int n_panels) {
    int nu = n_panels / 2;
    int nl = n_panels - nu;
    std::vector<double> xu(nu + 1), yu(nu + 1), xl(nl + 1), yl(nl + 1);
    for (int i = 0; i <= nu; ++i) {
        double s = static_cast<double>(i) / nu;
        xu[i] = 0.5 * (1.0 - std::cos(kPi * s));
        yu[i] = prof.upper(xu[i]);
    }
    xu[0] = 0.0;
    xu[nu] = 1.0;
    yu[nu] = prof.upper(1.0);
    for (int i = 0; i <= nl; ++i) {
        double s = static_cast<double>(i) / nl;
        xl[i] = 0.5 * (1.0 + std::cos(kPi * s));  // s=0 -> x=1.0 (TE) ... s=1 -> x=0.0 (LE)
        (void)0;
        yl[i] = prof.lower(xl[i]);
    }
    xl[nl] = 0.0;
    double yle = 0.5 * (yu[0] + yl[0]);
    yu[0] = yle;
    yl[nl] = yle;
    std::vector<double> px, py;
    for (int i = 0; i <= nu; ++i) {
        px.push_back(xu[i]);
        py.push_back(yu[i]);
    }
    for (int i = 1; i <= nl; ++i) {
        px.push_back(xl[i]);
        py.push_back(yl[i]);
    }
    std::vector<HSPanel> P;
    for (int i = 0; i + 1 < static_cast<int>(px.size()); ++i) {
        if (std::hypot(px[i + 1] - px[i], py[i + 1] - py[i]) < 1e-9) continue;
        HSPanel p;
        p.x1 = px[i];
        p.y1 = py[i];
        p.x2 = px[i + 1];
        p.y2 = py[i + 1];
        p.mx = 0.5 * (p.x1 + p.x2);
        p.my = 0.5 * (p.y1 + p.y2);
        p.ds = std::hypot(p.x2 - p.x1, p.y2 - p.y1);
        p.tx = (p.x2 - p.x1) / p.ds;
        p.ty = (p.y2 - p.y1) / p.ds;
        p.nx = -p.ty;
        p.ny = p.tx;
        p.upper = (i < nu);
        p.theta = std::atan2(p.y2 - p.y1, p.x2 - p.x1);
        P.push_back(p);
    }
    return P;
}

void sheet_coords(double x, double y, const HSPanel& p, double& xl, double& yl) {
    xl = (x - p.x1) * p.tx + (y - p.y1) * p.ty;
    yl = -(x - p.x1) * p.ty + (y - p.y1) * p.tx;
}

double subtended_angle(double xl, double yl, double c) {
    return std::atan2(yl * c, xl * (xl - c) + yl * yl);
}

void source_vel(double x, double y, const HSPanel& p, double q, double& u,
                double& v) {
    double xl, yl;
    sheet_coords(x, y, p, xl, yl);
    double c = p.ds;
    double r1 = std::max(std::hypot(xl, yl), 1e-10);
    double r2 = std::max(std::hypot(xl - c, yl), 1e-10);
    double th1 = std::atan2(yl, xl);
    double th2 = std::atan2(yl, xl - c);
    double ul = (q / (2.0 * kPi)) * std::log(r1 / r2);
    double vl = (q / (2.0 * kPi)) * (th2 - th1);
    u = ul * p.tx + vl * p.nx;
    v = ul * p.ty + vl * p.ny;
}

void vortex_vel(double x, double y, const HSPanel& p, double g, double& u,
                double& v) {
    double xl, yl;
    sheet_coords(x, y, p, xl, yl);
    double c = p.ds;
    double r1 = std::max(std::hypot(xl, yl), 1e-10);
    double r2 = std::max(std::hypot(xl - c, yl), 1e-10);
    double th1 = std::atan2(yl, xl);
    double th2 = std::atan2(yl, xl - c);
    double ul = (g / (2.0 * kPi)) * (th1 - th2);
    double vl = (g / (2.0 * kPi)) * std::log(r1 / r2);
    u = ul * p.tx + vl * p.nx;
    v = ul * p.ty + vl * p.ny;
}

}  // namespace

CaseResult solve_case(const Profile& prof, double alpha_deg, double reynolds,
                      double mach, const XFOptions& opt,
                      XFDistributions* dist_out) {
    CaseResult res;
    res.alpha_deg = alpha_deg;
    res.reynolds = reynolds;
    res.mach = mach;
    auto P = build_hs_panels(prof, opt.n_panels);
    int np = static_cast<int>(P.size());
    int nu = static_cast<int>(std::count_if(P.begin(), P.end(),
                                            [](const HSPanel& p) { return p.upper; }));
    double alpha = deg_to_rad(alpha_deg);
    double ca = std::cos(alpha), sa = std::sin(alpha);

    std::vector<std::vector<double>> An(np, std::vector<double>(np, 0.0));
    std::vector<std::vector<double>> Bn(np, std::vector<double>(np, 0.0));
    std::vector<std::vector<double>> At(np, std::vector<double>(np, 0.0));
    std::vector<std::vector<double>> Bt(np, std::vector<double>(np, 0.0));
    for (int i = 0; i < np; ++i) {
        for (int j = 0; j < np; ++j) {
            if (i == j) {
                An[i][j] = 0.5;
                Bn[i][j] = 0.0;
                At[i][j] = 0.0;
                Bt[i][j] = -0.5;
                continue;
            }
            double u, v;
            source_vel(P[i].mx, P[i].my, P[j], 1.0, u, v);
            An[i][j] = u * P[i].nx + v * P[i].ny;
            At[i][j] = u * P[i].tx + v * P[i].ty;
            vortex_vel(P[i].mx, P[i].my, P[j], 1.0, u, v);
            Bn[i][j] = u * P[i].nx + v * P[i].ny;
            Bt[i][j] = u * P[i].tx + v * P[i].ty;
        }
    }

    std::vector<double> sigma(np, 0.0);
    double gamma = 0.0;
    std::vector<double> ue(np, 0.0);
    std::vector<double> dstar(np, 0.0);
    std::vector<double> x_trans(2, 1.0);
    std::vector<double> x_sep(2, 1.0);

    auto solve_flow = [&]() {
        int n_unk = np + 1;
        std::vector<std::vector<double>> M(n_unk,
                                           std::vector<double>(n_unk + 1, 0.0));
        for (int i = 0; i < np; ++i) {
            for (int j = 0; j < np; ++j) M[i][j] = An[i][j];
            for (int j = 0; j < np; ++j) M[i][np] += Bn[i][j];
            M[i][n_unk] = -(ca * P[i].nx + sa * P[i].ny);
        }
        int kt = nu - 1;
        int kb = nu;
        for (int j = 0; j < np; ++j) {
            M[np][j] = At[kt][j] + At[kb][j];
            M[np][np] += Bt[kt][j] + Bt[kb][j];
        }
        M[np][n_unk] = -((ca * P[kt].tx + sa * P[kt].ty) +
                         (ca * P[kb].tx + sa * P[kb].ty));
        for (int col = 0; col < n_unk; ++col) {
            int piv = col;
            double best = std::fabs(M[col][col]);
            for (int r = col + 1; r < n_unk; ++r) {
                if (std::fabs(M[r][col]) > best) {
                    best = std::fabs(M[r][col]);
                    piv = r;
                }
            }
            if (best < 1e-13) continue;
            if (piv != col) std::swap(M[col], M[piv]);
            double d = M[col][col];
            for (int r = 0; r < n_unk; ++r) {
                if (r == col) continue;
                double f = M[r][col] / d;
                if (f == 0.0) continue;
                for (int c2 = col; c2 <= n_unk; ++c2) M[r][c2] -= f * M[col][c2];
            }
        }
        for (int i = 0; i < np; ++i)
            sigma[i] = (std::fabs(M[i][i]) > 1e-13) ? M[i][n_unk] / M[i][i] : 0.0;
        gamma = (std::fabs(M[np][np]) > 1e-13) ? M[np][n_unk] / M[np][np] : 0.0;
        for (int i = 0; i < np; ++i) {
            double vt = ca * P[i].tx + sa * P[i].ty;
            double s = 0.0, b = 0.0;
            for (int j = 0; j < np; ++j) {
                s += At[i][j] * sigma[j];
                b += Bt[i][j] * gamma;
            }
            ue[i] = vt + s + b;
        }
    };

    solve_flow();

    for (int iter = 0; iter < opt.max_iter; ++iter) {
        std::vector<double> sigma_new(np, 0.0);
        for (int side = 0; side < 2; ++side) {
            std::vector<double> xs, spd, idx;
            for (int i = 0; i < np; ++i) {
                if (P[i].upper == (side == 0)) {
                    xs.push_back(P[i].mx);
                    spd.push_back(std::fabs(ue[i]));
                    idx.push_back(i);
                }
            }
            if (xs.size() < 4) continue;
            double ue_max = 0.0;
            for (double s_ : spd) ue_max = std::max(ue_max, s_);
            double ue_floor = 0.15 * ue_max;
            for (double& s_ : spd) s_ = std::max(s_, ue_floor);
            SampledFunction fn;
            if (side == 1) {
                std::reverse(xs.begin(), xs.end());
                std::reverse(spd.begin(), spd.end());
                std::reverse(idx.begin(), idx.end());
            }
            fn.xs = xs;
            fn.ys = spd;
            fn.build(false);
            double xtr = (side == 0) ? opt.xtr_top : opt.xtr_bot;
            auto bl = bl_solve_surface(fn, 0.0, reynolds, 0.0, xtr,
                                       opt.fully_turbulent);
            x_trans[side] = bl.x_trans;
            x_sep[side] = bl.x_sep;
            for (size_t k = 0; k < bl.entries.size() && k < idx.size(); ++k) {
                dstar[idx[k]] = bl.entries[k].dstar;
            }
        }
        for (int i = 0; i < np; ++i) {
            int j_phys;
            if (P[i].upper)
                j_phys = (i + 1 < nu) ? i + 1 : i;
            else
                j_phys = (i > nu) ? i - 1 : i;
            double xj = P[j_phys].mx;
            double xi = P[i].mx;
            double dx = (P[i].upper) ? (xj - xi) : (xi - xj);
            double dd = (P[i].upper) ? (dstar[j_phys] - dstar[i])
                                      : (dstar[i] - dstar[j_phys]);
            if (std::fabs(dx) < 1e-9) dx = 1e-9;
            sigma_new[i] = std::fabs(ue[i]) * dd / dx;
        }
        double change = 0.0;
        for (int i = 0; i < np; ++i) {
            double s = sigma[i] * (1.0 - opt.relaxation) +
                       opt.relaxation * sigma_new[i];
            change = std::max(change, std::fabs(s - sigma[i]));
            sigma[i] = s;
        }
        solve_flow();
        if (change < opt.tol && iter > 0) break;
    }

    double fx_tot = 0.0, fy_tot = 0.0, cm = 0.0;
    for (int i = 0; i < np; ++i) {
        double ue_i = ue[i];
        double cp = 1.0 - ue_i * ue_i;
        double px_i = -cp * P[i].nx * P[i].ds;
        double py_i = -cp * P[i].ny * P[i].ds;
        fx_tot += px_i;
        fy_tot += py_i;
        cm += px_i * (P[i].my) - py_i * (P[i].mx);
    }
    cm = -cm;
    double cl = fy_tot * ca + fx_tot * sa;

    double cd_f = 0.0;
    double theta_te_top = 1e-4, theta_te_bot = 1e-4;
    double ue_te_top = 1.0, ue_te_bot = 1.0;
    for (int side = 0; side < 2; ++side) {
        std::vector<double> xs, spd, idx;
        for (int i = 0; i < np; ++i) {
            if (P[i].upper == (side == 0)) {
                xs.push_back(P[i].mx);
                spd.push_back(std::fabs(ue[i]));
                idx.push_back(i);
            }
        }
        if (xs.size() < 4) continue;
        double ue_max = 0.0;
        for (double s_ : spd) ue_max = std::max(ue_max, s_);
        double ue_floor = 0.15 * ue_max;
        for (double& s_ : spd) s_ = std::max(s_, ue_floor);
        SampledFunction fn;
        if (side == 1) {
            std::reverse(xs.begin(), xs.end());
            std::reverse(spd.begin(), spd.end());
            std::reverse(idx.begin(), idx.end());
        }
        fn.xs = xs;
        fn.ys = spd;
        fn.build(false);
        double xtr = (side == 0) ? opt.xtr_top : opt.xtr_bot;
        auto bl = bl_solve_surface(fn, 0.0, reynolds, 0.0, xtr,
                                   opt.fully_turbulent);
        for (size_t k = 1; k < bl.entries.size(); ++k) {
            double dxs = bl.entries[k].x - bl.entries[k - 1].x;
            cd_f += 0.5 * std::max(bl.entries[k].cf, 0.0) * dxs;
        }
        if (!bl.entries.empty()) {
            auto& last = bl.entries.back();
            if (side == 0) {
                theta_te_top = std::max(last.theta, 1e-6);
                ue_te_top = std::max(fn.eval(last.x), 0.05);
            } else {
                theta_te_bot = std::max(last.theta, 1e-6);
                ue_te_bot = std::max(fn.eval(last.x), 0.05);
            }
        }
        if (dist_out) {
            for (size_t k = 0; k < bl.entries.size() && k < idx.size(); ++k) {
                int i = idx[k];
                dist_out->x.push_back(P[i].mx);
                dist_out->cp.push_back(1.0 - ue[i] * ue[i]);
                dist_out->ue.push_back(std::fabs(ue[i]));
                dist_out->cf.push_back(bl.entries[k].cf);
                dist_out->theta.push_back(bl.entries[k].theta);
                dist_out->dstar.push_back(bl.entries[k].dstar);
                dist_out->side.push_back(side);
            }

        }
    }
    double ue_te_avg = 0.5 * (ue_te_top + ue_te_bot);
    double h_te = 2.55;
    double cd_sy_top = 2.0 * theta_te_top * std::pow(ue_te_top, (h_te + 5.0) / 6.0);
    double cd_sy_bot = 2.0 * theta_te_bot * std::pow(ue_te_bot, (h_te + 5.0) / 6.0);
    double cd_visc_sy = cd_sy_top + cd_sy_bot;
    double sep_top = x_sep[0], sep_bot = x_sep[1];
    double cl_atten = 1.0;
    if (sep_top < 0.9) {
        double f = (0.9 - sep_top) / 0.9;
        cl_atten = std::max(1.0 - 0.55 * f * f, 0.35);
    }
    if (mach > 0.02) {
        double beta = std::sqrt(std::max(1.0 - mach * mach, 0.05));
        cl = cl / beta * std::sqrt(std::max(1.0 - 0.08 * mach * mach, 0.5));
    }
    res.cl = cl * cl_atten;
    res.cd_friction = cd_f;
    res.cd_profil = std::max(cd_visc_sy, 1e-5);
    res.cd_pressure = std::max(res.cd_profil - cd_f, 0.0);
    res.cd = res.cd_profil;
    res.cm = cm;
    res.converged = true;
    res.trans.xtr_top = x_trans[0];
    res.trans.xtr_bot = x_trans[1];
    res.trans.xsep_top = x_sep[0];
    res.trans.xsep_bot = x_sep[1];
    res.trans.top_separated = x_sep[0] < 0.99;
    res.trans.bot_separated = x_sep[1] < 0.99;
    double cd_sep = 0.0;
    if (x_sep[0] < 0.98) cd_sep += 0.08 * std::pow(0.98 - x_sep[0], 1.5);
    if (x_sep[1] < 0.98) cd_sep += 0.08 * std::pow(0.98 - x_sep[1], 1.5);
    res.cd_profil += cd_sep;
    res.cd = res.cd_profil;
    (void)ue_te_avg;
    return res;
}

Polar solve_polar(const Profile& prof, double reynolds, double mach,
                  double alpha_min, double alpha_max, double alpha_step,
                  const XFOptions& opt, bool stop_on_stall,
                  std::vector<CaseResult>* raw) {
    Polar polar;
    double prev_cl = -1e9;
    for (double a = alpha_min; a <= alpha_max + 1e-9; a += alpha_step) {
        CaseResult r = solve_case(prof, a, reynolds, mach, opt);
        if (raw) raw->push_back(r);
        bool stalled = (stop_on_stall && prev_cl > -1e8 && r.cl < prev_cl - 0.05);
        PolarPoint pp;
        pp.alpha_deg = a;
        pp.cl = r.cl;
        pp.cd = r.cd;
        pp.cm = r.cm;
        pp.xtr_top = r.trans.xtr_top;
        pp.xtr_bot = r.trans.xtr_bot;
        polar.points.push_back(pp);
        if (stalled) break;
        prev_cl = r.cl;
    }
    return polar;
}

OptimizeResult optimize_profile(Profile& prof, const XFOptions& opt,
                                double reynolds, double mach, double alpha_min,
                                double alpha_max, double alpha_step) {
    OptimizeResult out;
    auto polar = solve_polar(prof, reynolds, mach, alpha_min, alpha_max,
                             alpha_step, opt, true);
    out.best_polar = polar;
    for (auto& p : polar.points) {
        if (p.cl > out.cl_max) {
            out.cl_max = p.cl;
            out.cd_at_clmax = p.cd;
            out.alpha_clmax = p.alpha_deg;
        }
        if (p.cd > 1e-8 && p.cl / p.cd > out.ld_max) {
            out.ld_max = p.cl / p.cd;
            out.cl_at_ldmax = p.cl;
            out.alpha_ldmax = p.alpha_deg;
        }
    }
    return out;
}

}  // namespace aerosim
