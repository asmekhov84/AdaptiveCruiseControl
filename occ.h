#pragma once

#ifndef OCC_20042013
#define OCC_20042013

#include "truck.h"
#include "pic.h"

class COrdinaryCruiseControl{
public:
	CTruck mdl;
	CPIController *picA_t, *picB_t;
	int G_m;
public:
	COrdinaryCruiseControl();
	void GetControlMode(double V, double &beta, double &B, int &G);
	int GetGear(double V);
};

#endif //OCC_20042013