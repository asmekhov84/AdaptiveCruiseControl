#include "truck.h"
#include "dataio.h"
#include "interp.h"
#include "common.h"
#include <windows.h>
#include <fstream>
#include <cmath>

CTruck::CTruck(){
	char buf[BUFSZ], fileName[BUFSZ];
	GetCurrentDirectoryA(BUFSZ, fileName);
	strcat_s(fileName, BUFSZ, "\\params.ini");
	// load world parameters
	GetPrivateProfileStringA("world", "g", NULL, buf, BUFSZ, fileName);
	g = atof(buf);
	GetPrivateProfileStringA("world", "ro_a", NULL, buf, BUFSZ, fileName);
	ro_a = atof(buf);
	GetPrivateProfileStringA("world", "ro_f", NULL, buf, BUFSZ, fileName);
	ro_f = atof(buf);
	// load vehicle parameters
	GetPrivateProfileStringA("vehicle", "c_x", NULL, buf, BUFSZ, fileName);
	c_x = atof(buf);
	GetPrivateProfileStringA("vehicle", "nu", NULL, buf, BUFSZ, fileName);
	nu = atof(buf);
	GetPrivateProfileStringA("vehicle", "phi", NULL, buf, BUFSZ, fileName);
	phi = atof(buf);
	GetPrivateProfileStringA("vehicle", "M", NULL, buf, BUFSZ, fileName);
	M = atof(buf);
	GetPrivateProfileStringA("vehicle", "R", NULL, buf, BUFSZ, fileName);
	R = atof(buf);
	GetPrivateProfileStringA("vehicle", "S_x", NULL, buf, BUFSZ, fileName);
	S_x = atof(buf);
	// load transmission parameters
	GetPrivateProfileStringA("transmission", "N_G", NULL, buf, BUFSZ, fileName);
	N_G = atoi(buf);
	GetPrivateProfileStringA("transmission", "G_map", NULL, buf, BUFSZ, fileName);
	G_map = new double[N_G + 1];
	G_map[0] = 0;
	char *p(buf);
	for(int i(1); i <= N_G; i++){
		G_map[i] = atof(p);
		p = strchr(p, ' ') + 1;
	}
	GetPrivateProfileStringA("transmission", "i_T", NULL, buf, BUFSZ, fileName);
	i_T = atof(buf);
	GetPrivateProfileStringA("transmission", "eta_T", NULL, buf, BUFSZ, fileName);
	eta_T = atof(buf);
	// load engine parameters
	FILE *fin = fopen("engine.dat", "r");
	MPC_trq_len = ReadFromDATFile(fin, "MPC_trq", MPC_trq);
	MPC_spd_len = ReadFromDATFile(fin, "MPC_spd", MPC_spd);
	ReadFromDATFile(fin, "MPC_map", MPC_map);
	FLC_len = ReadFromDATFile(fin, "FLC_spd", FLC_spd);
	ReadFromDATFile(fin, "FLC_map", FLC_map);
	n_min = FLC_spd[0];
	n_max = FLC_spd[FLC_len - 1];
	int brk_len;
	double *brk_spd, *brk_map;
	brk_len = ReadFromDATFile(fin, "brk_spd", brk_spd);
	ReadFromDATFile(fin, "brk_map", brk_map);
	fclose(fin);
	// calc. approx. polynom coeff.
	double x1(brk_spd[0]),
		y1(brk_map[0]),
		x2(brk_spd[brk_len / 2]),
		y2(brk_map[brk_len / 2]),
		x3(brk_spd[brk_len - 1]),
		y3(brk_map[brk_len - 1]);
	delete[] brk_spd;
	delete[] brk_map;
	brk_pln[2] = (y3 - (x3 * (y2 - y1) + x2 * y1 - x1 * y2) / (x2 - x1)) / (x3 * (x3 - x2 - x1) + x1 * x2);
	brk_pln[1] = (y2 - y1) / (x2 - x1) - brk_pln[2] * (x1 + x2);
	brk_pln[0] = (x2 * y1 - x1 * y2) / (x2 - x1) + brk_pln[2] * x1 * x2;
	// init. variables
	S = F = 0;
}

CTruck::~CTruck(){
	delete[] G_map;
	delete[] FLC_spd;
	delete[] FLC_map;
	delete[] MPC_spd;
	delete[] MPC_trq;
	delete[] MPC_map;
}

void CTruck::DoStep(double beta, double B, int G, double alpha, bool fuel){
	double n, sigma, f_f, f_fv, M_t, M_E,
		F_t, F_g, F_r, F_a, F_b, i_G;
	i_G = G_map[G];
	sigma = 1.05 + 0.07 * pow(i_T * i_G, 2.0);
	n = 30 * i_T * i_G * V / PI / R;
	// calc. torques and forces
	if(beta > 0){
		M_E = M_t = Interpolate1(FLC_spd, FLC_map, FLC_len, n) * beta;
	}
	else{
		M_t = 0;
		M_E = brk_pln[2] * n * n + brk_pln[1] * n + brk_pln[0];
	}
	F_t = M_E * i_T * i_G * eta_T / R;
	F_g = M * g * sin(alpha);
	F_r = nu * M * g * cos(alpha);
	F_a = 0.5 * c_x * S_x * ro_a * V * V;
	F_b = B * M * g * phi;
	// do one simulation step
	if(fuel){
		f_f = Interpolate2(MPC_spd, MPC_spd_len, MPC_trq, MPC_trq_len, MPC_map, n, M_t);
		f_fv = f_f / (3600000 * ro_f);
		F += M_t * n * f_fv * PI / 30 / V * L_SEG;
	}
	V += (F_t - F_g - F_r - F_a - F_b) / M / sigma / V * L_SEG; 
	S += L_SEG;
}