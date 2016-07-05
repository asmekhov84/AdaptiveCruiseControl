#include "occ.h"
#include "common.h"
#include "pic.h"

COrdinaryCruiseControl::COrdinaryCruiseControl(){
	picA_t = new CPIController(K_P_A, K_I_A, SE_A_MAX, 0, 1);
	picB_t = new CPIController(K_P_B, K_I_B, SE_B_MAX, 0, 1);
	G_m = 1;
}

void COrdinaryCruiseControl::GetControlMode(double V, double &beta, double &B, int &G){
	if(V > V_MAX){
		beta = 0;
		B = picB_t->GetOutput(V - V_MAX);
	}
	else{
		beta = picA_t->GetOutput(V_REF - V);
		B = 0;
	}
	G = GetGear(V);
}

int COrdinaryCruiseControl::GetGear(double V){
	double n;
	while(true){
		n = 30 * mdl.i_T * mdl.G_map[G_m] * V / PI / mdl.R;
		if(n < mdl.n_min && G_m > 1) G_m--;
		else if(n > mdl.n_max && G_m < mdl.N_G) G_m++;
		else break;
	}
	return G_m;
}