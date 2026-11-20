#pragma once
#include <string>
#include <vector>

namespace aerosim {

struct AircraftSpec {
    double mtow = 1500.0;
    double wing_area = 16.0;
    double wing_span = 12.0;
    double wing_chord = 1.333;
    double oswald = 0.8;
    double cd0 = 0.035;
    double cl_max_clean = 1.8;
    double cl_max_to = 2.4;
    double prop_efficiency = 0.8;
    double prop_diameter = 1.8;
    double prop_disk_area = 2.54;
    double static_thrust = 3000.0;
    double fuel_sfc = 0.35;
    double electric_power_max = 150000.0;
    double battery_energy = 40000.0;
    double battery_power_density = 500.0;
    double hybrid_share = 0.3;
    double motor_eff = 0.92;
    std::string name = "aircraft";
};

struct STOLResult {
    double v_stall;
    double v_stall_to;
    double v_lift_off;
    double v_loiter;
    double v_max_level;
    double ground_run;
    double total_takeoff;
    double total_landing;
    double landing_approach;
    double climb_rate;
    double service_ceiling;
    double range;
    double endurance;
    double ld_max;
    double cl_at_ldmax;
    double power_loading;
    double wing_loading;
    double disk_loading;
    double excess_thrust_hybrid;
    bool meets_150ft = false;
    bool meets_stol = false;
    double block_fuel_fraction;
    double battery_mass;
    double usable_fuel_mass;
};

STOLResult stol_analysis(const AircraftSpec& spec);
std::string stol_report(const AircraftSpec& spec, const STOLResult& r);

struct ParametricSweep {
    std::vector<double> x;
    std::vector<double> y;
    std::string xlabel, ylabel, title;
};

ParametricSweep sweep_wing_loading(const AircraftSpec& base, double wl_min,
                                   double wl_max, int n);
ParametricSweep sweep_power_loading(const AircraftSpec& base, double pl_min,
                                    double pl_max, int n);

}  // namespace aerosim
