#include "aircraft.h"
#include "common.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace aerosim {

static const double kRhoSL = 1.225;
static const double kG = 9.81;

STOLResult stol_analysis(const AircraftSpec& s) {
    STOLResult r;
    double W = s.mtow * kG;
    double AR = s.wing_span * s.wing_span / s.wing_area;
    double q_stall = W / (s.wing_area * s.cl_max_clean);
    r.v_stall = std::sqrt(2.0 * q_stall / kRhoSL);
    q_stall = W / (s.wing_area * s.cl_max_to);
    r.v_stall_to = std::sqrt(2.0 * q_stall / kRhoSL);
    r.v_lift_off = 1.1 * r.v_stall_to;
    r.wing_loading = W / s.wing_area;

    double induced = 1.0 / (kPi * AR * s.oswald);
    double cl_ld = std::sqrt(s.cd0 / induced);
    r.ld_max = std::sqrt(1.0 / (4.0 * s.cd0 * induced));
    r.cl_at_ldmax = cl_ld;

    double v_ldmax = std::sqrt(2.0 * W / (kRhoSL * s.wing_area * cl_ld));
    r.v_loiter = v_ldmax;
    q_stall = 0.5 * kRhoSL * r.v_loiter * r.v_loiter;
    double d_min = s.cd0 + induced * cl_ld * cl_ld;
    double p_req_loiter = d_min * q_stall * s.wing_area * r.v_loiter / s.prop_efficiency;
    (void)p_req_loiter;

    double p_shaft_total = s.static_thrust * r.v_lift_off / std::max(s.prop_efficiency, 0.1);
    double p_avail = std::min(p_shaft_total, s.electric_power_max + p_shaft_total * 0.7) * s.prop_efficiency;
    r.excess_thrust_hybrid = std::max(s.static_thrust - 0.12 * W, 0.0);
    double a_avg = 0.5 * p_avail / s.mtow;
    if (a_avg <= 0.1) a_avg = 0.1;
    if (a_avg > 4.0) a_avg = 4.0;
    r.ground_run = r.v_lift_off * r.v_lift_off / (2.0 * a_avg);
    r.ground_run = std::max(r.ground_run, 1.0);
    double h_screen = 15.0;
    double climb_angle = std::atan2(std::max(p_avail / (W * r.v_lift_off) - 1.0 / r.ld_max, 0.02), 1.0);
    double t_air = h_screen / std::max(r.v_lift_off * climb_angle, 0.5);
    r.total_takeoff = r.ground_run + r.v_lift_off * t_air;
    r.meets_150ft = r.total_takeoff <= 45.7;

    double v_approach = 1.15 * r.v_stall_to;
    r.landing_approach = v_approach;
    double decel = 0.35 * kG;
    r.total_landing = v_approach * v_approach / (2.0 * decel) + v_approach * 2.0;
    r.meets_stol = (r.total_landing <= 45.7) && (r.total_takeoff <= 45.7);

    double roc = p_avail / W - r.v_loiter / r.ld_max;
    r.climb_rate = std::max(roc, 0.0) * 60.0;

    double sigma_ceiling = 0.30;
    double rho_c = kRhoSL * sigma_ceiling;
    double q_c = 0.5 * rho_c * r.v_loiter * r.v_loiter;
    double p_req_c = d_min * q_c * s.wing_area * r.v_loiter / s.prop_efficiency;
    r.service_ceiling = p_avail > p_req_c ? 8000.0 : 4000.0;

    double battery_specific_energy = 200.0;
    r.battery_mass = s.battery_energy / battery_specific_energy;
    double v_cruise = 1.3 * r.v_stall;
    double q_cruise = 0.5 * kRhoSL * v_cruise * v_cruise;
    double cl_cruise = W / (q_cruise * s.wing_area);
    double cd_cruise = s.cd0 + induced * cl_cruise * cl_cruise;
    double d_cruise = cd_cruise * q_cruise * s.wing_area;
    double p_cruise = d_cruise * v_cruise / s.prop_efficiency;
    double tsfc = s.fuel_sfc / 1e6;
    r.usable_fuel_mass = std::max(s.mtow * 0.12, 120.0);
    double fuel_flow = p_cruise * tsfc;
    double endur_hr = r.usable_fuel_mass / std::max(fuel_flow, 1e-6) / 3600.0;
    endur_hr = std::min(std::max(endur_hr, 0.5), 24.0);
    double e_batt_j = s.battery_energy * 3600.0 * s.motor_eff * s.prop_efficiency;
    double p_electric_cruise = std::min(s.electric_power_max, p_cruise);
    double e_hours_elec = e_batt_j / std::max(p_cruise, 1.0) / 3600.0;
    e_hours_elec = std::min(e_hours_elec, endur_hr * 0.3);
    r.endurance = endur_hr + e_hours_elec;
    double range_fuel = v_cruise * endur_hr * 3.6;
    double range_electric = v_cruise * e_hours_elec * 3.6;
    r.range = range_fuel + range_electric;
    r.block_fuel_fraction = r.usable_fuel_mass / s.mtow;

    r.power_loading = s.mtow / (s.static_thrust / kG);
    r.disk_loading = s.static_thrust / s.prop_disk_area;
    return r;
}

std::string stol_report(const AircraftSpec& s, const STOLResult& r) {
    std::ostringstream o;
    o << "=== STOL Analysis: " << s.name << " ===\n";
    o << "MTOW: " << s.mtow << " kg | Wing: S=" << s.wing_area
      << " m2, b=" << s.wing_span << " m\n";
    o << "Wing loading: " << r.wing_loading << " N/m2\n";
    o << "V_stall (clean): " << r.v_stall << " m/s ("
      << r.v_stall * 1.944 << " kt)\n";
    o << "V_stall (TO config): " << r.v_stall_to << " m/s\n";
    o << "Ground run: " << r.ground_run << " m ("
      << r.ground_run * 3.281 << " ft)\n";
    o << "Total takeoff distance: " << r.total_takeoff << " m ("
      << r.total_takeoff * 3.281 << " ft) "
      << (r.meets_150ft ? "[MEETS 150ft]" : "[DOES NOT MEET]") << "\n";
    o << "Total landing distance: " << r.total_landing << " m ("
      << r.total_landing * 3.281 << " ft)\n";
    o << "L/D max: " << r.ld_max << " at Cl=" << r.cl_at_ldmax << "\n";
    o << "Rate of climb (hybrid boost): " << r.climb_rate << " m/min\n";
    o << "Range: " << r.range << " km | Endurance: " << r.endurance << " h\n";
    o << "Battery mass: " << r.battery_mass << " kg\n";
    o << "Power loading: " << r.power_loading << " kg/kW\n";
    o << "Disk loading: " << r.disk_loading << " N/m2\n";
    o << "STOL compliant: " << (r.meets_stol ? "YES" : "NO") << "\n";
    return o.str();
}

ParametricSweep sweep_wing_loading(const AircraftSpec& base, double wl_min,
                                   double wl_max, int n) {
    ParametricSweep sw;
    sw.xlabel = "Wing loading [kg/m2]";
    sw.ylabel = "Takeoff distance [m]";
    sw.title = "Takeoff distance vs wing loading";
    for (int i = 0; i < n; ++i) {
        double wl = wl_min + (wl_max - wl_min) * i / std::max(n - 1, 1);
        AircraftSpec s = base;
        s.wing_area = s.mtow / std::max(wl, 1.0);
        double span_ratio = 1.0;
        s.wing_span = std::sqrt(s.wing_area * (base.wing_span * base.wing_span / base.wing_area)) * span_ratio;
        auto r = stol_analysis(s);
        sw.x.push_back(wl);
        sw.y.push_back(r.total_takeoff);
    }
    return sw;
}

ParametricSweep sweep_power_loading(const AircraftSpec& base, double pl_min,
                                    double pl_max, int n) {
    ParametricSweep sw;
    sw.xlabel = "Static thrust [N]";
    sw.ylabel = "Takeoff distance [m]";
    sw.title = "Takeoff distance vs static thrust";
    for (int i = 0; i < n; ++i) {
        double st = pl_min + (pl_max - pl_min) * i / std::max(n - 1, 1);
        AircraftSpec s = base;
        s.static_thrust = st;
        auto r = stol_analysis(s);
        sw.x.push_back(st);
        sw.y.push_back(r.total_takeoff);
    }
    return sw;
}

}  // namespace aerosim
