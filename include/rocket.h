#ifndef ROCKET_H
#define ROCKET_H
#include "environment.h"

/**
 * @file rocket.h
 * @brief Declaration of the Rocket class for simulating rocket dynamics. 
 */
class Rocket {
private:
    double dryMass_;    // Mass of the rocket without fuel (kg)
    double position_;   // Vertical position (m)
    double velocity_;   // Vertical velocity (m/s)  
    double thrust_;     // Engine thrust (N)
    double fuel_;       // Remaining fuel mass (kg)
    double gravity_;    // Acceleration due to gravity (m/s^2)
    double fuelConsumptionRate_; // Fuel consumption rate per unit of thrust (kg/N/s)
    double area_;       // Cross-sectional area (m^2)
    double dragCoeff_;  // Drag coefficient

public:
    /**
     * @brief Construct a new Rocket object
     * 
     * @param dryMass Mass of the rocket without fuel (kg)
     * @param engineThrust Engine thrust (N)
     * @param initialFuel Initial fuel mass (kg)
     */
    Rocket(double dryMass, double engineThrust, double initialFuel, double gravity, double fuelConsumptionRate, double area, double dragCoeff);

    /**
     * @brief Update the rocket's state over a time step
     * 
     * @param dt Time step duration (s)
     */
    void step(double dt, const Environment& env);

    // --- Getters ---

    /**
     * @brief Get the current position of the rocket
     * 
     * @return Current vertical position (m)
     */
    double getPosition() const { return position_; }
    /**
     * @brief Get the current velocity of the rocket
     * 
     * @return Current vertical velocity (m/s)
     */
    double getVelocity() const { return velocity_; }
    /**
     * @brief Get the remaining fuel mass of the rocket
     * 
     * @return Remaining fuel mass (kg)
     */
    double getFuel() const { return fuel_; }
};

#endif // ROCKET_H