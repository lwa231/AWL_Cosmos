#include "core/ias15.h"

namespace cosmos {

namespace ias15 {

// TODO (step B): fill in the 8 Radau nodes
const double h[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// TODO (step B)
void gFromF(const double[8], double[8]) {}

// TODO (step B)
void bFromG(const double[8], double[7]) {}

} // namespace ias15

// TODO (step D)
int stepIAS15Fixed(BodyState&, const std::vector<double>&, Accel&, double)
{
    return 0;
}

} // namespace cosmos
