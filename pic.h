#pragma once

#ifndef PIC_20042013
#define PIC_20042013

class CPIController{
public:
	const double K_P, // proportional gain
		K_I, // integral gain
		sE_max, // saturation
		minOut, maxOut;
	double sE;
public:
	CPIController(double _K_P, double _K_I, double _sE_max, double _minOut, double _maxOut);
	double GetOutput(double E);
};

#endif //PIC_20042013