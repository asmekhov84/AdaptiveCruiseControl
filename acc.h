#pragma once

#ifndef ACC_17032013
#define ACC_17032013

#include "truck.h"
#include "road.h"
#include "pic.h"

class CAdaptiveCruiseControl{
public:
	CTruck mdl;
	CPIController *picA_m, *picB_m,
		*picA_t, *picB_t;
	// segmented road profile
	int N_seg; // number of nodes
	double *S_seg; // coordinate
	double *H_seg; // height
	double *A_seg; // angle
	int *T_seg; // type
	double *V_seg; // ref. velocity
	// threshold angles
	double alpha_up;
	double alpha_dn;
	int curSegNum;
	double L_sw;
public:
	CAdaptiveCruiseControl(const CRoadProfile *road);
	void GetControlMode(double S, double V, double &beta, double &B, int &G);
	int GetGear(int segType, double V);
};

#endif //ECC_17032013