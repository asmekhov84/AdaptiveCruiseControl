#include "acc.h"
#include "fis.h"
#include "road.h"
#include "pic.h"
#include "common.h"
#include "interp.h"
#include <windows.h>
#include <cmath>

CAdaptiveCruiseControl::CAdaptiveCruiseControl(const CRoadProfile *road){
	// calc. average uphills lenght
	int i;
	double L_up(0), L_up_max(0);
	for(i = 0; i < road->N - 1; i++){
		if(road->A[i] > 0){
			L_up += L_SEG;
			if(road->A[i + 1] <= 0 && L_up > L_up_max){
				L_up_max = L_up;
				L_up = 0;
			}
		}
	}
	if(L_up_max == 0) L_up_max = L_up;
	// calculate uphill threshold angle
	int G;
	for(alpha_up = 0; alpha_up < 5.0 / 180.0 * PI; alpha_up += 0.001){
		mdl.S = 0;
		mdl.V = V_REF;
		while(mdl.S < L_up_max){
			G = GetGear(0, mdl.V);
			mdl.DoStep(1, 0, G, alpha_up, false);
			if(mdl.V < V_MIN) break;
		}
		if(mdl.V < V_MIN) break;
	}
	// calculate downhill threshold angle
	double n, f_b, F_t;
	G = GetGear(-1, V_MIN);
	n = 30 * mdl.i_T * mdl.G_map[G] * V_MIN / PI / mdl.R;
	f_b = mdl.brk_pln[2] * n * n + mdl.brk_pln[1] * n + mdl.brk_pln[0];
	F_t = f_b * mdl.i_T * mdl.G_map[G] * mdl.eta_T / mdl.R;
	alpha_dn = (F_t - mdl.nu * mdl.M * mdl.g - 0.5 * mdl.c_x * mdl.S_x * mdl.ro_a * V_MIN * V_MIN) / mdl.M / mdl.g;
	// segmentation
	const int N(road->N);
	const double eps(0.5 / 180 * PI);
	int j(0), t1, t2;
	double *S_tmp(new double[N]),
		*H_tmp(new double[N]),
		*A_tmp(new double[N]),
		alpha_min(road->A[0]),
		alpha_max(road->A[0]);
	S_tmp[0] = road->S[0];
	if(road->A[0] >= alpha_up) t1 = 1;
	else if(road->A[0] <= alpha_dn) t1 = -1;
	else t1 = 0;
	i = 0;
	N_seg = 0;
	for(j = i + 1; j < N; j++){
		if(road->A[j] > alpha_max) alpha_max = road->A[j];
		else if(road->A[j] < alpha_min) alpha_min = road->A[j];
		if(road->A[j] >= alpha_up) t2 = 1;
		else if(road->A[j] <= alpha_dn) t2 = -1;
    else t2 = 0;
		if(alpha_max - alpha_min > eps || t2 != t1 || j == N - 1){
			H_tmp[N_seg] = road->H[i];
			A_tmp[N_seg] = road->A[i];
			S_tmp[N_seg + 1] = road->S[j];
			N_seg++;
			i = j;
			t1 = t2;
			alpha_min = road->A[i];
			alpha_max = road->A[i];
		}
	}
	H_tmp[N_seg] = road->H[N - 1];
	A_tmp[N_seg] = road->A[N - 1];
	N_seg++;
	S_seg = new double[N_seg];
	H_seg = new double[N_seg];
	A_seg = new double[N_seg];
	memcpy(S_seg, S_tmp, sizeof(double) * N_seg);
	memcpy(H_seg, H_tmp, sizeof(double) * N_seg);
	memcpy(A_seg, A_tmp, sizeof(double) * N_seg);
	delete[] S_tmp;
	delete[] H_tmp;
	delete[] A_tmp;
	T_seg = new int[N_seg];
	for(int i(0); i < N_seg; i++){
		T_seg[i] = (A_seg[i] >= alpha_up) - (A_seg[i] <= alpha_dn);
	}
	// calc. ref. velocities
	CFuzzyInferenceSystem fis;
	fis.Load("acc.fis");
	double inputs[3];
	V_seg = new double[N_seg];
	V_seg[N_seg - 1] = V_REF;
	for(int i(N_seg - 2); i >=0; i--){
		inputs[0] = V_seg[i + 1] * 3.6;
		inputs[1] = A_seg[i] * 180 / PI;
		inputs[2] = S_seg[i + 1] - S_seg[i];
		V_seg[i] = fis.GetValue(inputs) / 3.6;
	}
	//
	picA_m = new CPIController(K_P_A, K_I_A, SE_A_MAX, 0, 1);
	picB_m = new CPIController(K_P_B, K_I_B, SE_B_MAX, 0, 1);
	picA_t = new CPIController(K_P_A, K_I_A, SE_A_MAX, 0, 1);
	picB_t = new CPIController(K_P_B, K_I_B, SE_B_MAX, 0, 1);
	curSegNum = -1;
	L_sw = 0;
}

void CAdaptiveCruiseControl::GetControlMode(double S, double V, double &beta, double &B, int &G){
	double E;
	if(S_seg[curSegNum + 1] <= S){
		while(S_seg[curSegNum + 1] <= S) curSegNum++;
		mdl.S = S;
		mdl.V = V;
		picA_m->sE = picA_t->sE;
		picB_m->sE = picB_t->sE;
		while(abs(V_seg[curSegNum + 1] - mdl.V) > 0.5 / 3.6){
			E = V_seg[curSegNum + 1] - mdl.V;
			beta = picA_m->GetOutput(E);
			E = mdl.V - V_MAX;
			B = picB_m->GetOutput(E);
			G = GetGear(T_seg[curSegNum], mdl.V);
			mdl.DoStep(beta, B, G, A_seg[curSegNum], false);
			if(mdl.S >= S_seg[curSegNum + 1]) break;
		}
		L_sw = mdl.S - S;
	}
	int ctrlSegNum;
	ctrlSegNum = curSegNum;
	if(S >= S_seg[curSegNum + 1] - L_sw) ctrlSegNum++;
	beta = picA_t->GetOutput(V_seg[ctrlSegNum] - V);
	B = picB_t->GetOutput(V - V_MAX);
	G = GetGear(T_seg[curSegNum], V);
}

int CAdaptiveCruiseControl::GetGear(int segType, double V){
	int G;
	double n;
	if(segType == -1 && V <= V_MAX) segType = 0;
	switch(segType){
		case -1:
			for(G = 1; G <= mdl.N_G; G++){
				n = 30 * mdl.i_T * mdl.G_map[G] * V / PI / mdl.R;
				if(n <= mdl.n_max) break;
			}
			break;
		case 0:
			for(G = mdl.N_G; G >= 1; G--){
				n = 30 * mdl.i_T * mdl.G_map[G] * V / PI / mdl.R;
				if(n >= mdl.n_min) break;
			}
			break;
		case 1:
			double F_t_max(0), i_G, M_E, F_t;
			for(int G_i(1); G_i <= mdl.N_G; G_i++){
				i_G = mdl.G_map[G_i];
				n = 30 * mdl.i_T * i_G * V / PI / mdl.R;
				M_E = Interpolate1(mdl.FLC_spd, mdl.FLC_map, mdl.FLC_len, n);
				F_t = M_E * mdl.i_T * i_G * mdl.eta_T / mdl.R;
				if(F_t > F_t_max && n >= mdl.n_min && n <= mdl.n_max){
					F_t_max = F_t;
					G = G_i;
				}
			}
			break;
	}
	return G;
}