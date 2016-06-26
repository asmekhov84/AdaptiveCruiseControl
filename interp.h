#pragma once

#ifndef INTERP_09012013
#define INTERP_09012013

double Interpolate1(const double *x, const double *y, int l, double xi);
double Interpolate2(const double *x, int xl, const double *y, int yl, const double *z, double xi, double yi);
double Mean(const double *val, int l1, int l2);

#endif // INTERP_09012013