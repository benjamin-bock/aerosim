#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "../include/naca.h"


NacaAirfoil::NacaAirfoil(const std::string& designation)
    : designation_(designation), family_(0), tc_(0.12), camberMax_(0.0), camberPos_(0.4),
      clDesign_(0.0), clAlpha_(2.0 * M_PI), cl0_(0.0), alpha0_(0.0), clMax_(1.3) {
    parse(designation);
    compute2DAero();
}

void NacaAirfoil::parse(const std::string& name) {
    // Strip "NACA" prefix and dashes, keep meaningful digits.
    std::string s;
    for (char c : name) {
        if (c >= '0' && c <= '9') s += c;
    }
    std::string lower;
    for (char c : name) lower += (char)std::tolower((unsigned char)c);
    bool six = (!s.empty() && s[0] == '6' && s.size() >= 5);

    if (six) {
        family_ = 6;
        // 63-209 -> "63209": series=6, 3=pressure min pos 30%, 2 low-pressure side,
        // 2 = design Cl 0.2, 09 = t/c 9%
        if (s.size() >= 6) {
            // 64(2)215 -> digits "642215": design Cl digit = s[2], thickness = last two
            clDesign_ = (s[2] - '0') / 10.0;
            tc_ = std::stod(s.substr(s.size() - 2)) / 100.0;
        } else if (s.size() == 5) {
            // 63-209 -> "63209": design Cl digit = s[2], thickness = last two
            clDesign_ = (s[2] - '0') / 10.0;
            tc_ = std::stod(s.substr(3)) / 100.0;
        }
        camberPos_ = 0.4;
        camberMax_ = 0.0125 * clDesign_ / 0.2; // a=0.8 mean line: y_max ~ 0.0125 per 0.2 of design Cl
    } else if (s.size() == 5) {
        family_ = 5;
        int pDigit = s[0] - '0';
        int mDigit = s[1] - '0';
        camberPos_ = 0.15; // 2xxxx and 3xxxx series both use p=0.15 mean line
        camberMax_ = mDigit / 200.0; // L=2 -> 1.0% camber, L=3 -> 1.5%, matching Cl ~ 0.3/0.45
        clDesign_ = 0.15 * mDigit * (pDigit == 2 ? 1.0 : 1.5);
        tc_ = std::stod(s.substr(3)) / 100.0;
    } else if (s.size() == 4) {
        family_ = 4;
        camberMax_ = (s[0] - '0') / 100.0;
        camberPos_ = (s[1] - '0') / 10.0;
        clDesign_ = 0.115 * (s[0] - '0') * (s[1] - '0'); // Cl at design alpha
        tc_ = std::stod(s.substr(2)) / 100.0;
    } else {
        std::fprintf(stderr, "Warning: unrecognized airfoil '%s', defaulting to NACA 2412\n", name.c_str());
        family_ = 4;
        camberMax_ = 0.02;
        camberPos_ = 0.4;
        clDesign_ = 0.23;
        tc_ = 0.12;
    }
    if (tc_ < 0.03 || tc_ > 0.25) tc_ = 0.12;
}

double NacaAirfoil::camberAt(double xc) const {
    // Two-parabola mean line: exact for 4-digit, standard conceptual-design
    // approximation for 5-digit and 6-series mean lines.
    double m = camberMax_;
    double p = camberPos_;
    if (m <= 0.0) return 0.0;
    if (xc < p) return m / (p * p) * (2.0 * p * xc - xc * xc);
    return m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * xc - xc * xc);
}

double NacaAirfoil::thicknessAt(double xc) const {
    // Standard NACA thickness distribution (4/5-digit form)
    double a0 = 0.2969, a1 = -0.1260, a2 = -0.3516, a3 = 0.2843;
    // Closed trailing edge variant (a4 = -0.1015); open TE uses -0.1036
    double a4 = -0.1015;
    return 5.0 * tc_ * (a0 * std::sqrt(xc) + a1 * xc + a2 * xc * xc + a3 * xc * xc * xc + a4 * xc * xc * xc * xc);
}

void NacaAirfoil::compute2DAero() {
    // Thin-airfoil theory: alpha_L0 = -(1/pi) * integral( dy_c/dx * dtheta ), with
    // dtheta = dx/(sqrt(x(1-x))), integrated on the camber line.
    const int n = 2000;
    double integral = 0.0;
    for (int i = 0; i < n; ++i) {
        double theta = (i + 0.5) * M_PI / n;
        double x = (1.0 - std::cos(theta)) / 2.0;
        double dycam = (camberAt(x + 1e-5) - camberAt(x - 1e-5)) / 2e-5;
        integral += dycam * (1.0 - std::cos(theta)) * M_PI / n;
    }
    alpha0_ = integral / M_PI;
    cl0_ = -clAlpha_ * alpha0_;
    if (family_ == 6) {
        cl0_ = clDesign_;
        alpha0_ = -clDesign_ / clAlpha_;
    }

    // Empirical CLmax: base for t/c in 0.09-0.15 band, corrected by camber
    double tcRef = tc_ / 0.12;
    double base = 1.25 * std::sqrt(std::max(tcRef, 0.4));
    double camberTerm = 11.0 * camberMax_;
    if (family_ == 6) camberTerm = 10.0 * camberMax_ + 0.05 * clDesign_;
    clMax_ = std::min(base + camberTerm, 2.0);
}
