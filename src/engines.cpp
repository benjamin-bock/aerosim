#include "../include/engines.h"
#include <algorithm>

const std::vector<EngineSpec>& engineDatabase() {
    static const std::vector<EngineSpec> db = {
        {"Rotax 582",        48.5,  32.6, 0.36, "piston 2-stroke", 2},
        {"Rotax 912 UL",     59.6,  56.6, 0.30, "piston 4-stroke", 4},
        {"Rotax 912 iS",     73.5,  62.0, 0.27, "piston 4-stroke EFI", 4},
        {"Rotax 914 UL",     84.5,  64.0, 0.28, "piston turbo", 4},
        {"Continental O-200", 75.0,  76.0, 0.32, "piston", 4},
        {"Lycoming O-235",   86.0,  86.0, 0.32, "piston", 4},
        {"Lycoming O-320",  110.0, 100.0, 0.31, "piston", 4},
        {"Lycoming O-360",  134.0, 118.0, 0.30, "piston", 4},
        {"Lycoming IO-390", 145.0, 130.0, 0.30, "piston", 4},
        {"Continental IO-550", 205.0, 178.0, 0.29, "piston", 6},
        {"Lycoming IO-540", 235.0, 187.0, 0.29, "piston", 6},
        {"Continental TD-300", 224.0, 190.0, 0.24, "diesel", 4},
        {"TP-100 turboprop", 100.0, 90.0, 0.42, "turboprop", -1},
        {"PT6A-20",         430.0, 170.0, 0.40, "turboprop", -1},
        {"PT6A-27",         500.0, 180.0, 0.40, "turboprop", -1},
        {"PT6A-42",         635.0, 210.0, 0.39, "turboprop", -1},
        {"Allison 250-C20R", 317.0, 160.0, 0.42, "turboprop", -1},
    };
    return db;
}

int selectEngine(double requiredPowerKW, double margin) {
    const auto& db = engineDatabase();
    double need = requiredPowerKW * margin;
    int best = -1;
    double bestPower = 1e12;
    for (int i = 0; i < (int)db.size(); ++i) {
        if (db[i].powerKW >= need && db[i].powerKW < bestPower) {
            bestPower = db[i].powerKW;
            best = i;
        }
    }
    return best;
}
