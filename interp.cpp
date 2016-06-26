#include "interp.h"

// linear interpolation
double Interpolate1(const double *x, const double *y, int l, double xi){
	int i(0);
	while(xi >= x[i] && i < l - 1) i++;
	return (y[i] - y[i - 1]) / (x[i] - x[i - 1]) * (xi - x[i - 1]) + y[i - 1];
}

// bilinear interpolation
double Interpolate2(const double *x, int xl, const double *y, int yl, const double *z, double xi, double yi){
	int i(0), j(0);
	while(xi >= x[i] && i < xl - 1) i++;
	while(yi >= y[j] && j < yl - 1) j++;
	double r1 = ((x[i] - xi) / (x[i] - x[i - 1])) * z[j * xl + i - 1] + ((xi - x[i - 1]) / (x[i] - x[i - 1])) * z[j * xl + i];
	double r2 = ((x[i] - xi) / (x[i] - x[i - 1])) * z[(j - 1) * xl + i - 1] + ((xi - x[i - 1]) / (x[i] - x[i - 1])) * z[(j - 1) * xl + i];
	return (y[j - 1] - yi) / (y[j - 1] - y[j]) * r1 + (yi - y[j]) / (y[j - 1] - y[j]) * r2;
}

double Mean(const double *x, int l1, int l2){
	double avg(0);
	for(int i(l1); i <= l2; i++) avg += x[i];
	return avg / (l2 - l1 + 1);
}