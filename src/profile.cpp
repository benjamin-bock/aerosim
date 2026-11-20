#include "profile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aerosim {

void Profile::ensure_oriented() {
    if (xt_.size() < 3 || xb_.size() < 3) return;
    if (xt_.front() > xt_.back()) {
        std::reverse(xt_.begin(), xt_.end());
        std::reverse(yt_.begin(), yt_.end());
    }
    if (xb_.front() > xb_.back()) {
        std::reverse(xb_.begin(), xb_.end());
        std::reverse(yb_.begin(), yb_.end());
    }
}

void Profile::build_splines() {
    spline_coefs(xt_, yt_, sa_t_, sb_t_, sc_t_, sd_t_);
    spline_coefs(xb_, yb_, sa_b_, sb_b_, sc_b_, sd_b_);
}

void Profile::build_meta() {
    max_thickness_ = 0.0;
    max_camber_ = 0.0;
    int n = static_cast<int>(xt_.size());
    for (int i = 0; i < n; ++i) {
        double t = yt_[i] - yb_[i];
        if (t > max_thickness_) {
            max_thickness_ = t;
            x_max_thickness_ = xt_[i];
        }
        double cam = 0.5 * (yt_[i] + yb_[i]);
        if (std::fabs(cam) > std::fabs(max_camber_)) {
            max_camber_ = cam;
            x_max_camber_ = xt_[i];
        }
    }
    area_ = 0.0;
    for (int i = 0; i < n - 1; ++i) {
        double dx = xt_[i + 1] - xt_[i];
        area_ += 0.5 * (yt_[i] + yt_[i + 1]) * dx - 0.5 * (yb_[i] + yb_[i + 1]) * dx;
    }
    if (n >= 3) {
        double ytt = yt_[n - 2], ybb = yb_[n - 2];
        te_angle_ = rad_to_deg(std::atan2(ytt - ybb, xt_[n - 1] - xt_[n - 2]));
        te_gap_ = yt_[n - 1] - yb_[n - 1];
    }
}

Profile Profile::from_selig(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open profile file: " + path);
    std::string name;
    std::getline(f, name);
    if (name.empty()) name = "profile";
    std::vector<double> x, y;
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 3) continue;
        std::istringstream iss(line);
        double xv, yv;
        if (iss >> xv >> yv) {
            x.push_back(xv);
            y.push_back(yv);
        }
    }
    if (x.size() < 10) throw std::runtime_error("Profile file too small: " + path);
    Profile p;
    p.name_ = name;
    int imin = 0;
    for (int i = 1; i < static_cast<int>(x.size()); ++i)
        if (x[i] < x[imin]) imin = i;
    for (int i = 0; i <= imin; ++i) {
        p.xt_.push_back(x[i]);
        p.yt_.push_back(y[i]);
    }
    for (int i = static_cast<int>(x.size()) - 1; i >= imin; --i) {
        p.xb_.push_back(x[i]);
        p.yb_.push_back(y[i]);
    }
    p.ensure_oriented();
    p.build_splines();
    p.build_meta();
    return p;
}

static double naca_t(double x, double t) {
    double a4 = -0.1036;
    return 5.0 * t * (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x + a4 * x * x * x * x);
}

static double naca_camber_derivative(double x, double m, double p) {
    if (m == 0.0 || p == 0.0) return 0.0;
    if (x < p) return 2.0 * m / p * (p - x) / p;
    return 2.0 * m / (1.0 - p) * (p - x) / (1.0 - p);
}

static double naca_camber(double x, double m, double p) {
    if (m == 0.0 || p == 0.0) return 0.0;
    if (x < p) return m / (p * p) * (2.0 * p * x - x * x);
    return m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x);
}

Profile Profile::naca(std::string designation, bool verbose) {
    std::string d = designation;
    for (auto& ch : d) ch = static_cast<char>(std::tolower(ch));
    std::string digits;
    for (char ch : d)
        if (std::isdigit(static_cast<unsigned char>(ch))) digits += ch;
    double m = 0.0, p = 0.0, t = 0.0;
    if (digits.size() == 4) {
        m = (digits[0] - '0') / 100.0;
        p = (digits[1] - '0') / 10.0;
        t = std::stod(digits.substr(2)) / 100.0;
    } else if (digits.size() == 5) {
        t = std::stod(digits.substr(2)) / 100.0;
        double a = std::stod(digits.substr(0, 2)) / 20.0;
        m = 0.0;
        p = a;
    } else if (digits.size() == 6) {
        m = std::stod(digits.substr(0, 2)) / 100.0;
        p = std::stod(digits.substr(2, 2)) / 100.0;
        t = std::stod(digits.substr(4)) / 100.0;
    } else {
        throw std::runtime_error("Unsupported NACA designation: " + designation);
    }
    int n_side = 141;
    Profile prof;
    prof.name_ = "NACA " + digits;
    double eps = 1e-5;
    for (int i = 0; i < n_side; ++i) {
        double x = eps + (1.0 - 2.0 * eps) * i / (n_side - 1);
        if (i == n_side - 1) x = 1.0;
        if (i == 0) x = 0.0;
        double yc = naca_camber(x, m, p);
        double dyc = naca_camber_derivative(x, m, p);
        double th = std::atan(dyc);
        double yt = naca_t(x, t);
        prof.xt_.push_back(x - yt * std::sin(th));
        prof.yt_.push_back(yc + yt * std::cos(th));
        prof.xb_.push_back(x + yt * std::sin(th));
        prof.yb_.push_back(yc - yt * std::cos(th));
    }
    prof.build_splines();
    prof.build_meta();
    if (verbose) {
    }
    return prof;
}

double Profile::thickness(double x) const { return upper(x) - lower(x); }
double Profile::camber(double x) const { return 0.5 * (upper(x) + lower(x)); }
double Profile::upper(double x) const { return spline_eval(xt_, sa_t_, sb_t_, sc_t_, sd_t_, x); }
double Profile::lower(double x) const { return spline_eval(xb_, sa_b_, sb_b_, sc_b_, sd_b_, x); }

void Profile::rescale_thickness(double factor) {
    for (int i = 0; i < static_cast<int>(xt_.size()); ++i) {
        double xc = 0.5 * (xt_[i] + xb_[i]);
        double yc = 0.5 * (yt_[i] + yb_[i]);
        double dx = xt_[i] - xb_[i];
        double dy = yt_[i] - yb_[i];
        double th = std::atan2(dy, dx);
        double half = 0.5 * std::hypot(dx, dy) * factor;
        xt_[i] = xc - half * std::sin(th);
        yt_[i] = yc + half * std::cos(th);
        xb_[i] = xc + half * std::sin(th);
        yb_[i] = yc - half * std::cos(th);
    }
    build_splines();
    build_meta();
}

double Profile::upper_surface_derivative(double x) const {
    int n = static_cast<int>(xt_.size());
    if (n < 2) return 0.0;
    int i;
    if (x <= xt_[0])
        i = 0;
    else if (x >= xt_[n - 1])
        i = n - 2;
    else {
        int lo = 0, hi = n - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (xt_[mid] <= x)
                lo = mid + 1;
            else
                hi = mid;
        }
        i = std::max(0, lo - 1);
    }
    double dxp = xt_[i + 1] - xt_[i];
    if (dxp == 0.0) return 0.0;
    return (yt_[i + 1] - yt_[i]) / dxp;
}

double Profile::trailing_edge_gap() const { return te_gap_; }

void Profile::camber_flap(double x_hinge, double delta_deg) {
    double delta = deg_to_rad(delta_deg);
    auto deflect = [&](std::vector<double>& xs, std::vector<double>& ys) {
        for (int i = 0; i < static_cast<int>(xs.size()); ++i) {
            if (xs[i] > x_hinge) {
                double d = xs[i] - x_hinge;
                ys[i] += d * std::tan(delta);
            }
        }
    };
    deflect(xt_, yt_);
    deflect(xb_, yb_);
    build_splines();
    build_meta();
}

}  // namespace aerosim
