#include <cmath>
#include "../include/environment.h"

double Environment::getDensityAtAltitude(double altitude) const {
    return rho0 * std::exp(-altitude / scaleHeight);
}