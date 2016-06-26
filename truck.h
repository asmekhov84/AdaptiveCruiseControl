#pragma once

#ifndef TRUCK_16012013
#define TRUCK_16012013

class CTruck{
public:
	// world
	double g; // gravitational constant
	double ro_a; // air density
	double ro_f; // density of the diesel fuel
	// vehicle
	double c_x; // aerodynamic coeff.
	double nu; // rolling resistance coeff.
	double phi; // maximum friction force between wheel and road surface
	double M; // total mass
	double R; // wheel radius
	double S_x; // front section square
	// transmission
	int N_G; // count of gearbox ratios
	double *G_map; // array of gearbox ratios
	double i_T; // main ratio
	double eta_T; // transmission performance index
	// engine
	double n_min;	// min. engine speed
	double n_max; // max. engine speed
	int FLC_len;
	double *FLC_spd, *FLC_map;
	double brk_pln[3]; // brk. charact. approx. polynom
	int MPC_spd_len, MPC_trq_len;
	double *MPC_spd, *MPC_trq, *MPC_map;
	// state variables
	double S; // current coordinate
	double V; // current velocity
	double F; // current fuel consumption
public:
	CTruck();
	~CTruck();
	void DoStep(double beta, double B, int G, double alpha, bool fuel);
};

#endif //TRUCK_16012013