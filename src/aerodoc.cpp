#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "aircraft.h"
#include "profile.h"
#include "xf.h"

using namespace aerosim;

static void usage() {
    std::cout <<
        "AeroSim AeroDoc - airfoil & STOL design tool\n"
        "Usage: aerodoc <command> [options]\n\n"
        "Commands:\n"
        "  polar <profile> [options]     Compute a viscous polar (XF-like)\n"
        "  geom <profile>                Print profile geometry info\n"
        "  stol [options]                STOL/hybrid sizing analysis\n"
        "  sweep [options]               Parametric sweep\n"
        "  stol-batch <file>             Batch STOL analysis from CSV spec file\n"
        "\n"
        "Profile: NACA 4/5/6-digit designation, or path to a Selig .dat file.\n\n"
        "Polar options:\n"
        "  -re <n>       Reynolds number (default 1e6)\n"
        "  -mach <n>     Mach number (default 0)\n"
        "  -a0 <deg>     Start alpha (default -6)\n"
        "  -a1 <deg>     End alpha (default 18)\n"
        "  -da <deg>     Alpha step (default 0.5)\n"
        "  -n <n>        Panels per surface (default 140)\n"
        "  -xtrt <x>     Forced transition top (default free via Michel)\n"
        "  -xtrb <x>     Forced transition bottom\n"
        "  -turb         Fully turbulent\n"
        "  -csv <file>   Export polar to CSV\n"
        "  -summary      Print only key metrics\n"
        "\n"
        "STOL options:\n"
        "  -mtow <kg>    Max takeoff weight (default 1500)\n"
        "  -S <m2>       Wing area (default 16)\n"
        "  -b <m>        Wingspan (default 12)\n"
        "  -cd0 <n>      Parasitic drag (default 0.035)\n"
        "  -e <n>        Oswald efficiency (default 0.8)\n"
        "  -clmax <n>    Cl_max takeoff config (default 2.4)\n"
        "  -T <N>        Static thrust (default 3000)\n"
        "  -Pe <W>       Electric power (default 150000)\n"
        "  -Eb <Wh>      Battery energy (default 40000)\n"
        "  -hyb <0..1>   Hybridization share (default 0.3)\n";
}

static Profile load_profile(const std::string& arg) {
    if (arg.find('/') != std::string::npos ||
        arg.find('.') != std::string::npos) {
        return Profile::from_selig(arg);
    }
    return Profile::naca(arg);
}

static int cmd_polar(int argc, char** argv, int& i) {
    if (i >= argc) {
        std::cerr << "polar: missing profile\n";
        return 1;
    }
    Profile prof = load_profile(argv[i++]);
    double re = 1e6, mach = 0.0;
    double a0 = -6.0, a1 = 18.0, da = 0.5;
    XFOptions opt;
    std::string csv_out;
    bool summary = false;
    while (i < argc) {
        std::string a = argv[i];
        auto next = [&](double d) { return (i + 1 < argc) ? atof(argv[++i]) : d; };
        if (a == "-re") re = next(re);
        else if (a == "-mach") mach = next(mach);
        else if (a == "-a0") a0 = next(a0);
        else if (a == "-a1") a1 = next(a1);
        else if (a == "-da") da = next(da);
        else if (a == "-n") opt.n_panels = (int)next(opt.n_panels);
        else if (a == "-xtrt") opt.xtr_top = next(0.1);
        else if (a == "-xtrb") opt.xtr_bot = next(0.1);
        else if (a == "-turb") opt.fully_turbulent = true;
        else if (a == "-csv") csv_out = argv[++i];
        else if (a == "-summary") summary = true;
        else { std::cerr << "polar: unknown option " << a << "\n"; return 1; }
        ++i;
    }
    std::printf("# Profile: %s  Re=%.3g  Mach=%.2f\n", prof.name().c_str(), re, mach);
    std::printf("# alpha_deg      cl        cd       cm    xtr_top  xtr_bot\n");
    Polar p = solve_polar(prof, re, mach, a0, a1, da, opt, false);
    double cl_max = -1e9, a_clmax = 0, ld_max = 0, cl_ld = 0, cd_min = 1e9;
    std::ofstream csv;
    if (!csv_out.empty()) csv.open(csv_out);
    if (csv.is_open()) csv << "alpha_deg,cl,cd,cm,xtr_top,xtr_bot\n";
    for (auto& pt : p.points) {
        if (!summary) {
            std::printf("%10.3f %9.4f %9.5f %9.4f %8.3f %8.3f\n", pt.alpha_deg,
                        pt.cl, pt.cd, pt.cm, pt.xtr_top, pt.xtr_bot);
        }
        if (csv.is_open())
            csv << pt.alpha_deg << "," << pt.cl << "," << pt.cd << "," << pt.cm
                << "," << pt.xtr_top << "," << pt.xtr_bot << "\n";
        if (pt.cl > cl_max) { cl_max = pt.cl; a_clmax = pt.alpha_deg; }
        if (pt.cd > 1e-9 && pt.cl / pt.cd > ld_max) {
            ld_max = pt.cl / pt.cd;
            cl_ld = pt.cl;
        }
        if (pt.cd < cd_min && pt.alpha_deg > 0) cd_min = pt.cd;
    }
    double alpha0 = 0;
    for (size_t k = 1; k < p.points.size(); ++k) {
        if (p.points[k - 1].cl < 0 && p.points[k].cl >= 0) {
            double c1 = p.points[k - 1].cl, c2 = p.points[k].cl;
            double a1_ = p.points[k - 1].alpha_deg, a2_ = p.points[k].alpha_deg;
            alpha0 = a1_ + (0 - c1) * (a2_ - a1_) / std::max(c2 - c1, 1e-9);
        }
    }
    double slope = 0;
    for (size_t k = 1; k < p.points.size(); ++k) {
        if (p.points[k].alpha_deg >= -2 && p.points[k].alpha_deg <= 6) {
            slope = (p.points[k].cl - p.points[k - 1].cl) /
                    std::max(p.points[k].alpha_deg - p.points[k - 1].alpha_deg, 1e-9);
        }
    }
    std::printf("\n--- Summary ---\n");
    std::printf("Cl_max        = %.4f at alpha = %.1f deg\n", cl_max, a_clmax);
    std::printf("alpha_0       = %.2f deg\n", alpha0);
    std::printf("Cl_slope      = %.4f /deg\n", slope);
    std::printf("(L/D)_max     = %.1f at Cl = %.3f\n", ld_max, cl_ld);
    std::printf("Cd_min        = %.5f\n", cd_min);
    if (!csv_out.empty()) std::printf("CSV written to %s\n", csv_out.c_str());
    return 0;
}

static int cmd_geom(int argc, char** argv, int& i) {
    if (i >= argc) { std::cerr << "geom: missing profile\n"; return 1; }
    Profile prof = load_profile(argv[i++]);
    std::printf("Profile: %s\n", prof.name().c_str());
    std::printf("Max thickness    : %.4f at x=%.3f\n", prof.max_thickness(), prof.x_max_thickness());
    std::printf("Max camber       : %.4f at x=%.3f\n", prof.max_camber(), prof.x_max_camber());
    std::printf("Area             : %.5f\n", prof.area());
    std::printf("TE gap           : %.5f\n", prof.trailing_edge_gap());
    return 0;
}

static int cmd_stol(int argc, char** argv, int& i) {
    AircraftSpec spec;
    while (i < argc) {
        std::string a = argv[i];
        auto next = [&](double d) { return (i + 1 < argc) ? atof(argv[++i]) : d; };
        if (a == "-mtow") spec.mtow = next(spec.mtow);
        else if (a == "-S") spec.wing_area = next(spec.wing_area);
        else if (a == "-b") spec.wing_span = next(spec.wing_span);
        else if (a == "-cd0") spec.cd0 = next(spec.cd0);
        else if (a == "-e") spec.oswald = next(spec.oswald);
        else if (a == "-clmax") spec.cl_max_to = next(spec.cl_max_to);
        else if (a == "-T") spec.static_thrust = next(spec.static_thrust);
        else if (a == "-Pe") spec.electric_power_max = next(spec.electric_power_max);
        else if (a == "-Eb") spec.battery_energy = next(spec.battery_energy);
        else if (a == "-hyb") spec.hybrid_share = next(spec.hybrid_share);
        else { std::cerr << "stol: unknown option " << a << "\n"; return 1; }
        ++i;
    }
    auto r = stol_analysis(spec);
    std::cout << stol_report(spec, r);
    return 0;
}

static int cmd_sweep(int argc, char** argv, int& i) {
    AircraftSpec spec;
    std::string what = "wing";
    double lo = 60, hi = 200;
    int n = 20;
    while (i < argc) {
        std::string a = argv[i];
        auto next = [&](double d) { return (i + 1 < argc) ? atof(argv[++i]) : d; };
        if (a == "-what") what = argv[++i];
        else if (a == "-lo") lo = next(lo);
        else if (a == "-hi") hi = next(hi);
        else if (a == "-n") n = (int)next(n);
        else if (a == "-mtow") spec.mtow = next(spec.mtow);
        else if (a == "-S") spec.wing_area = next(spec.wing_area);
        else if (a == "-b") spec.wing_span = next(spec.wing_span);
        else if (a == "-T") spec.static_thrust = next(spec.static_thrust);
        else { std::cerr << "sweep: unknown option " << a << "\n"; return 1; }
        ++i;
    }
    ParametricSweep sw;
    if (what == "wing") sw = sweep_wing_loading(spec, lo, hi, n);
    else sw = sweep_power_loading(spec, lo, hi, n);
    std::printf("# %s\n# %s vs %s\n", sw.title.c_str(), sw.ylabel.c_str(), sw.xlabel.c_str());
    for (size_t k = 0; k < sw.x.size(); ++k)
        std::printf("%10.3f %12.4f\n", sw.x[k], sw.y[k]);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 1; }
    std::string cmd = argv[1];
    int i = 2;
    if (cmd == "polar") return cmd_polar(argc, argv, i);
    if (cmd == "geom") return cmd_geom(argc, argv, i);
    if (cmd == "stol") return cmd_stol(argc, argv, i);
    if (cmd == "sweep") return cmd_sweep(argc, argv, i);
    if (cmd == "stol-batch") {
        if (i >= argc) { std::cerr << "stol-batch: missing file\n"; return 1; }
        std::ifstream f(argv[i]);
        if (!f) { std::cerr << "cannot open " << argv[i] << "\n"; return 1; }
        std::string line;
        std::getline(f, line);
        std::printf("name,mtow,S,b,cd0,e,clmax,T,Pe,Eb,hyb,TO_dist_m,LAND_dist_m,Vstall,L/Dmax,STOL_ok\n");
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            AircraftSpec spec;
            std::string name;
            std::getline(ss, name, ',');
            spec.name = name;
            auto getd = [&](double d) { std::string t; std::getline(ss, t, ','); return t.empty() ? d : atof(t.c_str()); };
            spec.mtow = getd(spec.mtow);
            spec.wing_area = getd(spec.wing_area);
            spec.wing_span = getd(spec.wing_span);
            spec.cd0 = getd(spec.cd0);
            spec.oswald = getd(spec.oswald);
            spec.cl_max_to = getd(spec.cl_max_to);
            spec.static_thrust = getd(spec.static_thrust);
            spec.electric_power_max = getd(spec.electric_power_max);
            spec.battery_energy = getd(spec.battery_energy);
            spec.hybrid_share = getd(spec.hybrid_share);
            auto r = stol_analysis(spec);
            std::printf("%s,%.0f,%.1f,%.1f,%.4f,%.2f,%.2f,%.0f,%.0f,%.0f,%.2f,%.1f,%.1f,%.1f,%.1f,%s\n",
                        name.c_str(), spec.mtow, spec.wing_area, spec.wing_span,
                        spec.cd0, spec.oswald, spec.cl_max_to, spec.static_thrust,
                        spec.electric_power_max, spec.battery_energy,
                        spec.hybrid_share, r.total_takeoff, r.total_landing,
                        r.v_stall, r.ld_max, r.meets_stol ? "YES" : "NO");
        }
        return 0;
    }
    usage();
    return 1;
}
