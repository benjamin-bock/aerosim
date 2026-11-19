#ifndef PROPELLER_H
#define PROPELLER_H

/**
 * @file propeller.h
 * @brief Fixed/variable pitch propeller model: efficiency eta = f(J) from a
 * Clark-Y two-blade experimental fit, static thrust, power model.
 */

struct Propeller {
    double diameterM = 1.8;
    int nBlades = 2;
    bool variablePitch = true;
    double activityFactor = 100.0; // blade activity factor

    // Advance ratio J = V/(nD), n in rev/s
    // Efficiency fit (variable pitch, cruise prop), valid 0 < J < 1.4+
    double efficiency(double J) const;

    // Static thrust (momentum theory with disk-area limit + figure of merit)
    double staticThrust(double powerW) const;

    // Thrust at speed with given shaft power: T = P*eta(J)/V (V>0)
    // falls back to static thrust at low speed
    double thrust(double powerW, double speedMs, double rpm) const;

    // Recommended diameter from power (Raymer ch. 10 rule of thumb)
    static double recommendedDiameter(double powerKW, bool variablePitch);
};

#endif
