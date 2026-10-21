#include "common.h"

#include <algorithm>
#include <stdexcept>

namespace aerosim {

double deg_to_rad(double deg) { return deg * kPi / 180.0; }
double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

std::vector<double> lin_space(double x0, double x1, int n) {
    std::vector<double> v(n);
    if (n <= 1) return v;
    double dx = (x1 - x0) / (n - 1);
    for (int i = 0; i < n; ++i) v[i] = x0 + i * dx;
    v[n - 1] = x1;
    return v;
}

void spline_coefs(const std::vector<double>& x, const std::vector<double>& y,
                  std::vector<double>& a, std::vector<double>& b,
                  std::vector<double>& c, std::vector<double>& d) {
    int n = static_cast<int>(x.size()) - 1;
    a.assign(n + 1, 0.0);
    b.assign(n, 0.0);
    c.assign(n + 1, 0.0);
    d.assign(n, 0.0);
    if (n < 1) return;
    if (n == 1) {
        a[0] = y[0];
        a[1] = y[1];
        b[0] = (y[1] - y[0]) / (x[1] - x[0]);
        return;
    }
    std::vector<double> h(n), alpha(n), l(n + 1), mu(n), z(n + 1);
    for (int i = 0; i < n; ++i) h[i] = x[i + 1] - x[i];
    for (int i = 1; i < n; ++i)
        alpha[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    for (int i = 1; i < n; ++i) {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    l[n] = 1.0;
    z[n] = 0.0;
    c[n] = 0.0;
    for (int j = n - 1; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
    }
    for (int i = 0; i <= n; ++i) a[i] = y[i];
}

double spline_eval(const std::vector<double>& x, const std::vector<double>& a,
                   const std::vector<double>& b, const std::vector<double>& c,
                   const std::vector<double>& d, double xv) {
    int n = static_cast<int>(x.size()) - 1;
    if (n < 1) return a.empty() ? 0.0 : a[0];
    int i = n - 1;
    if (xv >= x[n]) {
        double dx = xv - x[n - 1];
        return a[n - 1] + b[n - 1] * dx + c[n - 1] * dx * dx + d[n - 1] * dx * dx * dx;
    }
    int lo = 0, hi = n;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (x[mid] <= xv)
            lo = mid + 1;
        else
            hi = mid;
    }
    i = std::max(0, lo - 1);
    double dx = xv - x[i];
    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
}

void SampledFunction::build(bool close_kutta) {
    int n = static_cast<int>(xs.size());
    sa = xs;
    sb.assign(n, 0.0);
    sc.assign(n, 0.0);
    sd.assign(n, 0.0);
    if (n < 2) return;
    std::vector<double> h(n), sub(n), diag(n), sup(n), rhs(n);
    for (int i = 0; i < n - 1; ++i) h[i] = xs[i + 1] - xs[i];
    if (!close_kutta) {
        diag[0] = 1.0;
        sub[0] = 0.0;
        sup[0] = 0.0;
        rhs[0] = (ys[1] - ys[0]) / h[0];
        diag[n - 1] = 1.0;
        sub[n - 1] = 0.0;
        sup[n - 1] = 0.0;
        rhs[n - 1] = (ys[n - 1] - ys[n - 2]) / h[n - 2];
    } else {
        diag[0] = 1.0;
        sub[0] = -1.0;
        sup[0] = 0.0;
        rhs[0] = 0.0;
        diag[n - 1] = 1.0;
        sub[n - 1] = 0.0;
        sup[n - 1] = 0.0;
        rhs[n - 1] = (ys[n - 1] - ys[n - 2]) / h[n - 2];
    }
    for (int i = 1; i < n - 1; ++i) {
        sub[i] = h[i];
        diag[i] = 2.0 * (h[i - 1] + h[i]);
        sup[i] = h[i - 1];
        rhs[i] = 3.0 * ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1]);
    }
    std::vector<double> cp(n);
    for (int i = 0; i < n; ++i) {
        double m = diag[i];
        if (i > 0 && sub[i] != 0.0) {
            m -= sub[i] * sup[i - 1] / diag[i - 1];
            rhs[i] -= sub[i] * rhs[i - 1] / diag[i - 1];
        }
        diag[i] = m;
        cp[i] = m;
    }
    std::vector<double> m(n);
    for (int i = n - 1; i >= 0; --i) {
        if (i == n - 1) {
            m[i] = rhs[i] / diag[i];
        } else {
            m[i] = (rhs[i] - sup[i] * m[i + 1]) / diag[i];
        }
    }
    for (int i = 0; i < n; ++i) {
        sa[i] = m[i];
        if (i < n - 1) {
            sb[i] = (ys[i + 1] - ys[i]) / h[i] - h[i] * (m[i + 1] + 2.0 * m[i]) / 3.0;
            sd[i] = (m[i + 1] - m[i]) / (3.0 * h[i]);
        } else {
            sb[i] = m[i] * xs[i] + ys[i] - m[i] * xs[i];
            sd[i] = 0.0;
        }
        sc[i] = ys[i];
    }
    a0 = ys.empty() ? 0.0 : ys[0];
}

double SampledFunction::eval(double x) const {
    int n = static_cast<int>(xs.size());
    if (n == 0) return a0;
    if (n == 1) return sc[0];
    int i = n - 1;
    if (x < xs[0]) i = 0;
    else if (x >= xs[n - 1]) i = n - 1;
    else {
        int lo = 0, hi = n - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (xs[mid] <= x)
                lo = mid + 1;
            else
                hi = mid;
        }
        i = std::max(0, lo - 1);
    }
    if (i >= n - 1) i = n - 2;
    double dx = x - xs[i];
    double si = sc[i] + sb[i] * dx + sa[i] * dx * dx + sd[i] * dx * dx * dx;
    return si;
}

double SampledFunction::deriv(double x) const {
    int n = static_cast<int>(xs.size());
    if (n < 2) return 0.0;
    int i;
    if (x <= xs[0])
        i = 0;
    else if (x >= xs[n - 1])
        i = n - 2;
    else {
        int lo = 0, hi = n - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (xs[mid] <= x)
                lo = mid + 1;
            else
                hi = mid;
        }
        i = std::max(0, lo - 1);
    }
    double dx = x - xs[i];
    return sb[i] + 2.0 * sa[i] * dx + 3.0 * sd[i] * dx * dx;
}

double SampledFunction::integrate(double x0, double x1) const {
    int n = static_cast<int>(xs.size());
    if (n < 2) return ys.empty() ? 0.0 : ys[0] * (x1 - x0);
    int steps = 200;
    double sum = 0.0;
    double h = (x1 - x0) / steps;
    for (int k = 0; k < steps; ++k) {
        double xa = x0 + k * h;
        sum += (eval(xa) + eval(xa + h)) * 0.5 * h;
    }
    return sum;
}

}  // namespace aerosim
