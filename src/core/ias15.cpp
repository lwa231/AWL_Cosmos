#include "core/ias15.h"

namespace cosmos {

namespace ias15 {

// Left Gauss-Radau nodes on [0, 1] (8 points, exact quadrature to degree 14), computed at 40 digits.
const double h[8] = {
    0.0,
    0.05626256053692214646565219,
    0.1802406917368923649875799,
    0.3526247171131696373739078,
    0.5471536263305553830014486,
    0.7342101772154105315232106,
    0.8853209468390957680903598,
    0.9775206135612875018911745};

namespace {
// c[m][j] = coefficient of tau^j in tau (tau - h1) ... (tau - h(m-1)); computed once from the nodes.
struct CTable {
    double c[8][8] = {};
    CTable()
    {
        c[1][1] = 1.0;
        for (int m = 2; m < 8; ++m) {
            c[m][m] = 1.0;
            c[m][1] = -h[m - 1] * c[m - 1][1];
            for (int j = 2; j < m; ++j) c[m][j] = c[m - 1][j - 1] - h[m - 1] * c[m - 1][j];
        }
    }
};
const CTable C;
} // namespace

void gFromF(const double F[8], double g[8])
{
    g[0] = 0.0;
    for (int k = 1; k < 8; ++k) {
        double t = (F[k] - F[0]) / h[k];
        for (int i = 1; i < k; ++i) t = (t - g[i]) / (h[k] - h[i]);
        g[k] = t;
    }
}

void bFromG(const double g[8], double b[7])
{
    for (int j = 1; j < 8; ++j) {
        double s = 0.0;
        for (int m = j; m < 8; ++m) s += g[m] * C.c[m][j];
        b[j - 1] = s;
    }
}

double positionAt(double x0, double v0, double a0, const double b[7], double hh, double dt)
{
    double p = b[6] / 72.0;
    p = b[5] / 56.0 + hh * p;
    p = b[4] / 42.0 + hh * p;
    p = b[3] / 30.0 + hh * p;
    p = b[2] / 20.0 + hh * p;
    p = b[1] / 12.0 + hh * p;
    p = b[0] / 6.0 + hh * p;
    p = a0 / 2.0 + hh * p;
    return x0 + v0 * dt * hh + dt * dt * hh * hh * p;
}

double velocityAt(double v0, double a0, const double b[7], double hh, double dt)
{
    double p = b[6] / 8.0;
    p = b[5] / 7.0 + hh * p;
    p = b[4] / 6.0 + hh * p;
    p = b[3] / 5.0 + hh * p;
    p = b[2] / 4.0 + hh * p;
    p = b[1] / 3.0 + hh * p;
    p = b[0] / 2.0 + hh * p;
    p = a0 + hh * p;
    return v0 + dt * hh * p;
}

} // namespace ias15

// TODO (2e): predictor-corrector iteration
int stepIAS15Fixed(BodyState&, const std::vector<double>&, Accel&, double)
{
    return 0;
}

} // namespace cosmos
