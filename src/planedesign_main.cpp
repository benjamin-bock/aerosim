#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <string>
#include "../include/planedesign.h"
#include "../include/naca.h"

static void printResult(const Requirements& req, const DesignResult& r, const PlaneDesign& pd) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n================ AIRCRAFT CONCEPTUAL DESIGN (Raymer) ================\n";
    std::cout << "Airfoil: NACA " << pd.airfoil().designation()
              << "  (t/c = " << pd.airfoil().tc() * 100 << "%, clmax2D ~ "
              << pd.airfoil().clMax() << ")\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "WEIGHT SIZING (converged in " << r.iterations << " iterations)\n";
    std::cout << "  Max takeoff weight W0 : " << r.w0Kg << " kg\n";
    std::cout << "  Empty weight We        : " << r.weKg << " kg (" << r.weKg / r.w0Kg * 100 << "%)\n";
    std::cout << "  Fuel weight Wf         : " << r.wfKg << " kg (" << r.wfKg / r.w0Kg * 100 << "%)\n";
    std::cout << "  Payload + crew         : " << req.payloadKg + req.crewKg << " kg\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "GEOMETRY\n";
    std::cout << "  Wing area S            : " << r.wingAreaM2 << " m^2\n";
    std::cout << "  Wingspan b             : " << r.spanM << " m (AR = " << req.aspectRatio << ")\n";
    std::cout << "  Wing loading W/S       : " << r.w0Kg * 9.81 / r.wingAreaM2 << " N/m^2 ("
              << r.w0Kg / r.wingAreaM2 << " kg/m^2)\n";
    std::cout << "  CLmax clean / takeoff  : " << r.clmaxClean << " / " << r.clmax << "\n";
    std::cout << "  Stall speed (TO flap)  : " << r.stallSpeedMs << " m/s ("
              << r.stallSpeedMs * 1.94384 << " kt)\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "AERODYNAMICS\n";
    std::cout << "  CD0 (equivalent skin friction): " << r.cd0 << "\n";
    std::cout << "  Oswald efficiency e     : " << r.oswald << "\n";
    std::cout << "  K (induced)            : " << r.kInduced << "\n";
    std::cout << "  (L/D)max               : " << r.ldMax << "\n";
    std::cout << "  L/D at cruise          : " << r.cruiseLD << "\n";
    std::cout << "---------------------------------------------------------------------\n";
    std::cout << "PROPULSION SIZING (governing constraint)\n";
    std::cout << "  Required T/W           : " << r.twReq << "\n";
    std::cout << "  Required thrust        : " << r.thrustN << " N\n";
    if (req.propDriven) {
        std::cout << "  Required power P/W     : " << r.pwReq << " W/kg\n";
        std::cout << "  Required shaft power   : " << r.powerW << " W ("
                  << r.powerW / 745.7 << " hp)\n";
    }
    std::cout << "=====================================================================\n";
}

static void exportCsv(const std::string& path, const Requirements& req,
                      const DesignResult& r, const PlaneDesign& pd) {
    std::ofstream out(path);
    out << "parameter,value,unit\n";
    out << "W0," << r.w0Kg << ",kg\n";
    out << "We," << r.weKg << ",kg\n";
    out << "Wf," << r.wfKg << ",kg\n";
    out << "wing_area," << r.wingAreaM2 << ",m2\n";
    out << "span," << r.spanM << ",m\n";
    out << "CLmax_clean," << r.clmaxClean << ",-\n";
    out << "CLmax_takeoff," << r.clmax << ",-\n";
    out << "stall_speed," << r.stallSpeedMs << ",m/s\n";
    out << "CD0," << r.cd0 << ",-\n";
    out << "oswald," << r.oswald << ",-\n";
    out << "K," << r.kInduced << ",-\n";
    out << "LD_max," << r.ldMax << ",-\n";
    out << "LD_cruise," << r.cruiseLD << ",-\n";
    out << "TW_required," << r.twReq << ",-\n";
    out << "thrust_required," << r.thrustN << ",N\n";
    out << "power_required," << r.powerW << ",W\n";
    out << "airfoil," << pd.airfoil().designation() << ",-\n";
    out.close();
    std::cout << "Design summary exported to " << path << "\n";
}

static void exportConstraintCsv(const std::string& path, const PlaneDesign& pd,
                                double wsMin, double wsMax, int n) {
    auto pts = pd.constraintCurve(wsMin, wsMax, n);
    std::ofstream out(path);
    out << "ws_N_m2,tw_required,pw_required_W_kg,constraint\n";
    for (const auto& p : pts) {
        out << p.ws << "," << p.twJet << "," << p.pwProp << "," << p.constraint << "\n";
    }
    out.close();
    std::cout << "Constraint analysis exported to " << path << " ("
              << pts.size() << " points)\n";
}

int main(int argc, char** argv) {
    Requirements req;
    std::string csvPath = "output/design_summary.csv";
    std::string consPath = "output/constraint_analysis.csv";
    double wsMin = 300.0, wsMax = 3000.0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double& d) { if (i + 1 < argc) d = std::atof(argv[++i]); };
        auto nextStr = [&](std::string& s) { if (i + 1 < argc) s = argv[++i]; };
        if (a == "-h" || a == "--help") {
            std::cout << "PlaneDesign: Raymer-based conceptual aircraft design tool\n"
                      << "  -pay <kg>      payload\n"
                      << "  -crew <kg>     crew weight\n"
                      << "  -range <km>    cruise range\n"
                      << "  -vc <m/s>      cruise speed\n"
                      << "  -alt <m>       cruise altitude\n"
                      << "  -end <h>       loiter endurance\n"
                      << "  -vs <m/s>      max stall speed\n"
                      << "  -to <m>        takeoff distance (over obstacle)\n"
                      << "  -hobs <m>      obstacle height\n"
                      << "  -roc <m/s>     rate of climb at -vcl\n"
                      << "  -vcl <m/s>     climb speed\n"
                      << "  -vmax <m/s>    max level speed\n"
                      << "  -n <->         design load factor (0=skip)\n"
                      << "  -ar <->        aspect ratio\n"
                      << "  -af <name>     NACA airfoil (e.g. 2412, 23012, 63-209)\n"
                      << "  -jet           jet propulsion (default: prop)\n"
                      << "  -eta <->       propeller efficiency\n"
                      << "  -sfc <kg/kWh>  specific fuel consumption (prop)\n"
                      << "  -tsfc <1/h>    jet TSFC\n"
                      << "  -flap <dCL>    takeoff flap CLmax increment\n"
                      << "  -csv <path>    output design summary CSV\n"
                      << "  -conscsv <path> output constraint analysis CSV\n";
            return 0;
        }
        else if (a == "-pay") next(req.payloadKg);
        else if (a == "-crew") next(req.crewKg);
        else if (a == "-range") next(req.rangeKm);
        else if (a == "-vc") next(req.cruiseSpeedMs);
        else if (a == "-alt") next(req.cruiseAltM);
        else if (a == "-end") next(req.enduranceH);
        else if (a == "-vs") next(req.stallSpeedMs);
        else if (a == "-to") next(req.takeoffDistM);
        else if (a == "-hobs") next(req.obstacleHeightM);
        else if (a == "-roc") next(req.climbRateMs);
        else if (a == "-vcl") next(req.climbSpeedMs);
        else if (a == "-vmax") next(req.maxSpeedMs);
        else if (a == "-n") next(req.loadFactor);
        else if (a == "-ar") next(req.aspectRatio);
        else if (a == "-af") nextStr(req.airfoilName);
        else if (a == "-jet") req.propDriven = false;
        else if (a == "-eta") next(req.propEff);
        else if (a == "-sfc") next(req.sfcKgPerkWh);
        else if (a == "-tsfc") next(req.jetTsfc);
        else if (a == "-flap") next(req.clmaxFlapDelta);
        else if (a == "-csv") nextStr(csvPath);
        else if (a == "-conscsv") nextStr(consPath);
    }

    std::cout << "--- AeroSim PlaneDesign (Raymer conceptual sizing) ---\n";
    PlaneDesign pd(req);
    DesignResult r = pd.synthesize();
    printResult(req, r, pd);
    exportCsv(csvPath, req, r, pd);
    exportConstraintCsv(consPath, pd, wsMin, wsMax, 60);
    return 0;
}
