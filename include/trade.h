#ifndef TRADE_H
#define TRADE_H

#include <vector>
#include "synthesis.h"

/**
 * @file trade.h
 * @brief Parametric trade studies: AR x wing-loading carpet plot of W0,
 * minimum-weight optimum search (Raymer ch. 18-19).
 */
struct TradePoint {
    double ar, ws, w0, powerKW, ldMax;
    bool converged;
};

struct TradeStudyResult {
    std::vector<TradePoint> grid;   // rows: ws, cols: ar
    std::vector<double> arValues;
    std::vector<double> wsValues;
    TradePoint optimum;
    bool found = false;
};

TradeStudyResult carpetStudy(const DesignSpec& baseSpec,
                             const std::vector<double>& arValues,
                             const std::vector<double>& wsValues,
                             double wsStallOverride = 0.0);

#endif
