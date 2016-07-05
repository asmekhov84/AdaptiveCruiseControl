#include "pic.h"
#include "common.h"

#define min(a, b) a <= b ? a : b
#define max(a, b) a >= b ? a : b

CPIController::CPIController(double _K_P, double _K_I, double _sE_max, double _minOut, double _maxOut):
	K_P(_K_P), K_I(_K_I), sE_max(_sE_max), minOut(_minOut), maxOut(_maxOut){
	sE = 0;
}

double CPIController::GetOutput(double E){
	sE += E * L_SEG;
	sE = min(max(sE, -sE_max), sE_max);
	double y;
	y = K_P * E + K_I * sE;
	return min(max(y, minOut), maxOut);
}