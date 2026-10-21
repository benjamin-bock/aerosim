#ifndef NACA_H
#define NACA_H

#include <string>

/**
 * @file naca.h
 * @brief NACA airfoil family: geometry and low-order aerodynamic estimates.
 *
 * Supports NACA 4-digit (e.g. 2412), 5-digit (e.g. 23012) and 6-series
 * (e.g. 63-209, 64(2)215). Estimates use thin-airfoil theory integrated on
 * the exact camber line, plus empirical correlations for CLmax suitable for
 * conceptual design (Raymer, Aircraft Design: A Conceptual Approach, ch. 4 & 12).
 */
class NacaAirfoil {
private:
    std::string designation_;
    int family_;        // 4, 5 or 6
    double tc_;         // thickness ratio t/c
    double camberMax_;  // max camber ratio (y/c)
    double camberPos_;   // position of max camber (x/c)
    double clDesign_;   // ideal/design lift coefficient (6-series) or thin-airfoil Cl at alpha=0
    double clAlpha_;    // 2D lift-curve slope (per rad)
    double cl0_;        // 2D lift at zero alpha
    double alpha0_;    // zero-lift angle of attack (rad, negative nose-down for camber)
    double clMax_;      // 2D max lift coefficient estimate

    void parse(const std::string& name);
    void compute2DAero();

public:
    explicit NacaAirfoil(const std::string& designation);

    // Camber line ordinate at x/c (0..1)
    double camberAt(double xc) const;
    // Thickness distribution half-ordinate at x/c
    double thicknessAt(double xc) const;

    const std::string& designation() const { return designation_; }
    double tc() const { return tc_; }
    double camberMax() const { return camberMax_; }
    double clAlpha() const { return clAlpha_; }
    double cl0() const { return cl0_; }
    double alpha0() const { return alpha0_; }
    double clMax() const { return clMax_; }
};

#endif
