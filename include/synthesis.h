#ifndef SYNTHESIS_H
#define SYNTHESIS_H

/**
 * @file synthesis.h
 * @brief Full iterative design synthesis: couples weight, aero, geometry,
 * constraints, performance and engine selection in a damped fixed-point loop.
 */
#include "requirements2.h"
#include "geometry.h"
#include "aero.h"
#include "weights.h"
#include "constraints.h"
#include "performance.h"
#include "engines.h"

struct SynthesisResult {
    bool converged = false;
    int iterations = 0;
    double w0Kg = 0, weKg = 0, wfKg = 0;
    double wingArea = 0, span = 0, aspectRatio = 0;
    double powerKW = 0;
    double tw = 0, pw = 0;
    const char* governingConstraint = "";
    Geometry geo;
    AeroResult aero;
    WeightResult weights;
    PerformanceResult perf;
    ConstraintSet cset;
    int engineIdx = -1;
    std::vector<double> w0History;
};

SynthesisResult synthesizeDesign(const DesignSpec& spec);

#endif
