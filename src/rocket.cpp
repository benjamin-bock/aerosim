#include <cmath>
#include "../include/rocket.h"
#include "../include/constants.h"

Rocket::Rocket(double dryMass, double engineThrust, double initialFuel, double gravity, double fuelConsumptionRate, double area, double dragCoeff)
    : dryMass_(dryMass), thrust_(engineThrust), fuel_(initialFuel), gravity_(gravity), fuelConsumptionRate_(fuelConsumptionRate), area_(area), dragCoeff_(dragCoeff) {
        position_ = 0.0;
        velocity_ = 0.0;
    }

void Rocket::step(double dt, const Environment& env) {
    double speedOfSound = 340.0; // Setting speed of sound at sea level (m/s)
    double mach = std::abs(velocity_) / speedOfSound;

    // Modeling drag coefficient variation with Mach number
    double currentCd;
    if (mach < 0.8) {
        currentCd = dragCoeff_; // Subsonic value (e.g., 0.4)
    } else if (mach < 1.2) {
        // Transonic peak (linearly increasing up to 2x the value)
        currentCd = dragCoeff_ * (1.0 + 2.5 * (mach - 0.8));
    } else {
        // Supersonic (decreasing with 1/Mach)
        currentCd = (dragCoeff_ * 2.0) / std::sqrt(mach);
    }

    double currentThrust = 0.0;
    double rho = env.getDensityAtAltitude(position_);
    double dragForce = 0.5 * rho * velocity_ * std::abs(velocity_) * currentCd * area_;

    if (fuel_ > 0) {
        double fuelConsumed = thrust_ * dt * fuelConsumptionRate_; // Simplified fuel consumption
        if (fuelConsumed > fuel_) {
            fuelConsumed = fuel_;
        }
        fuel_ -= fuelConsumed;
        currentThrust = thrust_;
    }

    double totalMass = dryMass_ + fuel_;
    double totalForce = currentThrust - dragForce - (totalMass * env.gravity); // Net force
    double acceleration = totalForce / totalMass;
    velocity_ += acceleration * dt;
    position_ += velocity_ * dt;
}

