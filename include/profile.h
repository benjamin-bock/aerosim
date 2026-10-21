#pragma once
#include "common.h"

namespace aerosim {

struct PanelResult {
    std::vector<double> v;
    std::vector<double> cp;
    double cl;
    double cm;
};

struct PanelInput {
    std::vector<double> xt;
    std::vector<double> xb;
    std::vector<double> xc;
    std::vector<double> yc;
    std::vector<double> theta;
    std::vector<double> ds;
    int n_panels;
};

class Profile {
   public:
    static Profile from_selig(const std::string& path);
    static Profile naca(std::string designation, bool verbose = false);

    const std::vector<double>& xt() const { return xt_; }
    const std::vector<double>& yt() const { return yt_; }
    const std::vector<double>& xb() const { return xb_; }
    const std::vector<double>& yb() const { return yb_; }
    int n_points() const { return static_cast<int>(xt_.size()); }
    double max_thickness() const { return max_thickness_; }
    double max_camber() const { return max_camber_; }
    double x_max_thickness() const { return x_max_thickness_; }
    double x_max_camber() const { return x_max_camber_; }
    double area() const { return area_; }
    double leading_edge_radius() const { return le_radius_; }
    double trailing_edge_angle_deg() const { return te_angle_; }
    const std::string& name() const { return name_; }
    void set_name(const std::string& n) { name_ = n; }

    double thickness(double x) const;
    double camber(double x) const;
    double upper(double x) const;
    double lower(double x) const;
    void rescale_thickness(double factor);
    double upper_surface_derivative(double x) const;
    double trailing_edge_gap() const;
    void camber_flap(double x_hinge, double delta_deg);

   private:
    std::vector<double> xt_, yt_, xb_, yb_;
    std::vector<double> sa_t_, sb_t_, sc_t_, sd_t_;
    std::vector<double> sa_b_, sb_b_, sc_b_, sd_b_;
    std::string name_ = "profile";
    double max_thickness_ = 0.0, max_camber_ = 0.0;
    double x_max_thickness_ = 0.0, x_max_camber_ = 0.0;
    double area_ = 0.0, le_radius_ = 0.0, te_angle_ = 0.0, te_gap_ = 0.0;

    void build_meta();
    void build_splines();
    void ensure_oriented();
};

}  // namespace aerosim
