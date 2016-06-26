#include "road.h"
#include "interp.h"
#include "dataio.h"
#include "common.h"
#include <fstream>
#include <cmath>

#define min(a, b) a <= b ? a : b
#define max(a, b) a >= b ? a : b

CRoadProfile::CRoadProfile(char *profileName){
	char fileName[BUFSZ];
	// load road profile
	int N_src;
	double *S_src, *H_src, *A_src;
	strcpy_s(fileName, sizeof(char) * BUFSZ, "profiles\\");
	strcat_s(fileName, sizeof(char) * BUFSZ, profileName);
	strcat_s(fileName, sizeof(char) * BUFSZ, ".dat");
	std::ifstream fin(fileName);
	N_src = ReadFromDATFile(fin, "way", S_src);
	ReadFromDATFile(fin, "hgt", H_src);
	ReadFromDATFile(fin, "ang", A_src);
	fin.close();
	double L_way = S_src[N_src - 1];
	// interpolate road data on required range
	N = int(L_way / L_SEG + 1);
	S = new double[N];
	A = new double[N];
	for(int i(0); i < N; i++){
		S[i] = i * L_SEG;
		A[i] = Interpolate1(S_src, A_src, N_src, S[i]);
	}
	// smooth angles curve (running average)
	const int wndSz(3);
	int l1, l2;
	double *A_tmp = new double[N];
	memcpy(A_tmp, A, sizeof(double) * N);
	for(int i(0); i < N; i++){
		l1 = max(0, i - int(wndSz / 2));
		l2 = min(N - 1, i + int(wndSz / 2));
		A[i] = Mean(A_tmp, l1, l2);
	}
	delete[] A_tmp;
	// calc. heights based on angles
	H = new double[N];
	H[0] = H_src[0];
	for(int i(1); i < N; i++){
		H[i] = H[i - 1] + L_SEG * tan(A[i - 1]);
	}
	delete[] S_src;
	delete[] H_src;
	delete[] A_src;
}

CRoadProfile::~CRoadProfile(){
	delete[] S;
	delete[] H;
	delete[] A;
}

double CRoadProfile::GetH(double _S){
	return Interpolate1(S, H, N, _S);
}

double CRoadProfile::GetA(double _S){
	return Interpolate1(S, A, N, _S);
}