#include <iostream> // For console output
#include <fstream>  // For file output (CSV)
#include "../include/rocket.h" // Your class header
#include "../include/constants.h" // Default constants

int main(int argc, char** argv) {
    std::cout << "--- AeroSim v1.0 ---" << std::endl;
    std::cout << "Initializing simulation..." << std::endl;

    // Default parameters
    double dryMass = DEFAULT_DRY_MASS;
    double thrust = DEFAULT_THRUST;
    double fuel = DEFAULT_INITIAL_FUEL;
    double dt = DEFAULT_DT;
    double gravity = DEFAULT_GRAVITY;
    double fuelConsumptionRate = DEFAULT_FUEL_CONSUMPTION_RATE;
    double area = DEFAULT_AREA;
    double dragCoeff = DEFAULT_DRAG_COEFFICIENT;

    // Parsing command-line arguments
    // Start at i = 1 because argv[0] is the program name
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            std::cout << "AeroSim Usage:" << std::endl;
            std::cout << "  -m <value>    Set dry mass (kg)" << std::endl;
            std::cout << "  -t <value>    Set thrust (N)" << std::endl;
            std::cout << "  -f <value>    Set initial fuel mass (kg)" << std::endl;
            std::cout << "  -dt <value>   Set time step (s)" << std::endl; 
            std::cout << "  -g <value>    Set gravity (m/s^2)" << std::endl;
            std::cout << "  -fc <value>   Set fuel consumption rate (kg/N/s)" << std::endl;
            std::cout << "  -a <value>    Set cross-sectional area (m^2)" << std::endl;
            std::cout << "  -cd <value>   Set drag coefficient" << std::endl;
            return 0; // Exit after displaying help
        }

        // Check if the argument is a flag and if there is a value after it
        if (arg[0] == '-' && i + 1 < argc) {
            std::string value = argv[i + 1];
            
            if (arg == "-m") {
                dryMass = std::stod(value);
                if(dryMass < 0) {
                    std::cerr << "Error: Dry mass (-m) cannot be negative." << std::endl;
                    return 1;
                }
            } else if (arg == "-t") {
                thrust = std::stod(value);
                if(thrust < 0) {
                    std::cerr << "Error: Thrust (-t) cannot be negative." << std::endl;
                    return 1;
                }
            } else if (arg == "-f") {
                fuel = std::stod(value);
                if(fuel < 0) {
                    std::cerr << "Error: Initial fuel mass (-f) cannot be negative." << std::endl;
                    return 1;
                }
            } else if (arg == "-dt") {
                dt = std::stod(value);
                if(dt <= 0) {
                    std::cerr << "Error: Time step (-dt) must be positive." << std::endl;
                    return 1;
                }
            } else if (arg == "-g") {
                gravity = std::stod(value);
                if(gravity <= 0) {
                    std::cerr << "Error: Gravity (-g) must be positive." << std::endl;
                    return 1;
                }
            } else if (arg == "-fc") {
                fuelConsumptionRate = std::stod(value);
                if(fuelConsumptionRate < 0) {
                    std::cerr << "Error: Fuel consumption rate (-fc) cannot be negative." << std::endl;
                    return 1;
                }
            } else if (arg == "-a") {
                area = std::stod(value);
               if(area <= 0) {
                    std::cerr << "Error: Cross-sectional area (-a) must be positive." << std::endl;
                    return 1;
                }              
            } else if (arg == "-cd") {
                dragCoeff = std::stod(value);
                if(dragCoeff < 0) {
                    std::cerr << "Error: Drag coefficient (-cd) cannot be negative." << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Warning: Unknown argument " << arg << " (Ignored)" << std::endl;
            }
            
            // Skip the next element since we've just consumed it as a value
            i++; 
        }
    }

    // Display for verification
    std::cout << "AeroSim Configuration:" << std::endl;
    std::cout << "  Dry Mass (-m): " << dryMass << " kg";
    if(dryMass == DEFAULT_DRY_MASS) std::cout << " (Default)";
    std::cout << std::endl;
    std::cout << "  Thrust   (-t): " << thrust << " N";
    if(thrust == DEFAULT_THRUST) std::cout << " (Default)";
    std::cout << std::endl;
    std::cout << "  Fuel     (-f): " << fuel << " kg";
    if(fuel == DEFAULT_INITIAL_FUEL) std::cout << " (Default)";
    std::cout << std::endl;
    std::cout << "  Step     (-dt): " << dt << " s";
    if(dt == DEFAULT_DT) std::cout << " (Default)";
    std::cout << std::endl;
    std::cout << "  Gravity  (-g): " << gravity << " m/s^2";
    if(gravity == DEFAULT_GRAVITY) std::cout << " (Default)";
    std::cout << std::endl;
    std::cout << "  Fuel Consumption Rate (-fc): " << fuelConsumptionRate << " kg/N/s";
    if(fuelConsumptionRate == DEFAULT_FUEL_CONSUMPTION_RATE) std::cout << " (Default)";
    std::cout << std::endl;
    if(area == DEFAULT_AREA) std::cout << "  Area     (-a): " << area << " m^2 (Default)" << std::endl;
    else std::cout << "  Area     (-a): " << area << " m^2" << std::endl;
    if(dragCoeff == DEFAULT_DRAG_COEFFICIENT) std::cout << "  Drag Coeff (-cd): " << dragCoeff << " (Default)" << std::endl;
    else std::cout << "  Drag Coeff (-cd): " << dragCoeff << " " << std::endl;

    // Instantiate the Rocket object
    Rocket myRocket(dryMass, thrust, fuel, gravity, fuelConsumptionRate, area, dragCoeff);

    // Data Logging Setup
    std::ofstream dataFile("output/flight_data.csv");
    if (!dataFile.is_open()) {
        std::cerr << "Error: Could not open file for writing." << std::endl;
        return 1;
    }

    // Write CSV Header
    dataFile << "Time,Altitude,Velocity,Fuel" << std::endl;

    // Simulation Loop
    double currentTime = 0.0;
    const double MAX_SIM_TIME = 300.0; // Safety cutoff (5 minutes)

    std::cout << "Launch! Processing physics..." << std::endl;

    while (currentTime < MAX_SIM_TIME) {
        // Update Physics
        myRocket.step(dt, Environment(gravity));
        currentTime += dt;

        // Log Data
        dataFile << currentTime << "," 
                 << myRocket.getPosition() << "," 
                 << myRocket.getVelocity() << "," 
                 << myRocket.getFuel() << std::endl;

        // Check Stop Conditions
        // Stop if the rocket hits the ground AFTER launch (t > 1.0s)
        if (myRocket.getPosition() <= 0.0 && currentTime > 1.0) {
            std::cout << "Touchdown detected at t = " << currentTime << " s" << std::endl;
            // Force altitude to 0 for clean data
            dataFile << currentTime << ",0,0," << myRocket.getFuel() << std::endl;
            break;
        }
    }

    // Cleanup
    dataFile.close();
    std::cout << "Simulation finished. Data exported to 'flight_data.csv'." << std::endl;

    std::cout << "--- Results ---" << std::endl;
    std::cout << "Flight Time: " << currentTime << " s" << std::endl;

    return 0;
}