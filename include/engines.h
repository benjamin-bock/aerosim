#ifndef ENGINES_H
#define ENGINES_H

#include <vector>
#include <string>

/**
 * @file engines.h
 * @brief Off-the-shelf engine database and selection (piston/turboprop).
 */
struct EngineSpec {
    const char* model;
    double powerKW;      // rated shaft power (kW)
    double massKg;       // dry mass
    double sfcKgPerkWh;  // cruise SFC
    const char* type;    // piston / turboprop / diesel / electric
    int cylinders = 4;
};

const std::vector<EngineSpec>& engineDatabase();

// Pick smallest engine with power >= required (with installation margin),
// or the closest one. Returns index into database, -1 if none.
int selectEngine(double requiredPowerKW, double margin = 1.0);

#endif
