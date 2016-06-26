#include "common.h"
#include "truck.h"
#include "road.h"
#include "acc.h"
#include "occ.h"
#include "dataio.h"
#include "fis.h"
#include <fstream>
#include <iostream>
#include <windows.h>

using namespace std;

#define MSPERD 86400000
#define MSPERH 3600000
#define MSPERM 60000
#define MSPERS 1000

char *GetTimeDifference(SYSTEMTIME &t1, SYSTEMTIME &t2);

void main(){
	char profileName[BUFSZ];
	cout << "Enter road profile name: ";
	cin.getline(profileName, BUFSZ);

	CRoadProfile road(profileName);
	CTruck truck1, truck2;
	CAdaptiveCruiseControl acc(&road);
	COrdinaryCruiseControl occ;

	double S1, S2, S_max = road.S[road.N - 1];
	do{
		cout << "Start position [0, " << S_max << "]: ";
		cin >> S1;
	}
	while(S1 < 0 || S1 > S_max);
	do{
		cout << "Finish position (" << S1 << ", " << S_max << "]: ";
		cin >> S2;
	}
	while(S2 <= S1 || S2 > S_max);

	const int N((S2 - S1) / L_SEG + 1);
	int *G1(new int[N]),
		*G2(new int[N]), i;
	double *S(new double[N]),
		*H(new double[N]),
		*A(new double[N]),
		*V1(new double[N]),
		*V2(new double[N]),
		*F1(new double[N]),
		*F2(new double[N]),
		*A1(new double[N]),
		*A2(new double[N]),
		*B1(new double[N]),
		*B2(new double[N]),
		T1(0), T2(0);

	SYSTEMTIME startTime, finishTime;
	GetSystemTime(&startTime);
	i = 0;
	truck1.S = S1;
	truck1.V = V_REF;
	while(truck1.S + L_SEG <= S2){
		acc.GetControlMode(truck1.S, truck1.V, A1[i], B1[i], G1[i]);
		S[i] = truck1.S;
		H[i] = road.GetH(S[i]);
		A[i] = road.GetA(S[i]);
		V1[i] = truck1.V;
		F1[i] = truck1.F;
		T1 += L_SEG / V1[i];
		truck1.DoStep(A1[i], B1[i], G1[i], road.GetA(truck1.S), true);
		i++;
	}
	S[i] = truck1.S;
	H[i] = road.GetH(S[i]);
	A[i] = road.GetA(S[i]);
	V1[i] = V1[i - 1];
	F1[i] = F1[i - 1];
	A1[i] = A1[i - 1];
	B1[i] = B1[i - 1];
	G1[i] = G1[i - 1];

	i = 0;
	truck2.S = S1;
	truck2.V = V_REF;
	while(truck2.S + L_SEG <= S2){
		occ.GetControlMode(truck2.V, A2[i], B2[i], G2[i]);
		V2[i] = truck2.V;
		F2[i] = truck2.F;
		T2 += L_SEG / V2[i];
		truck2.DoStep(A2[i], B2[i], G2[i], road.GetA(truck2.S), true);
		i++;
	}
	V2[i] = V2[i - 1];
	F2[i] = F2[i - 1];
	A2[i] = A2[i - 1];
	B2[i] = B2[i - 1];
	G2[i] = G2[i - 1];
	GetSystemTime(&finishTime);
	char *dt;
	dt = GetTimeDifference(startTime, finishTime);

	ofstream fout("acc.m");
	WriteToMFile(fout, S, N, "S");
	WriteToMFile(fout, H, N, "H");
	WriteToMFile(fout, A, N, "A");
	WriteToMFile(fout, V1, N, "V1");
	WriteToMFile(fout, V2, N, "V2");
	WriteToMFile(fout, F1, N, "F1");
	WriteToMFile(fout, F2, N, "F2");
	WriteToMFile(fout, A1, N, "beta1");
	WriteToMFile(fout, A2, N, "beta2");
	WriteToMFile(fout, B1, N, "B1");
	WriteToMFile(fout, B2, N, "B2");
	WriteToMFile(fout, G1, N, "G1");
	WriteToMFile(fout, G2, N, "G2");
	fout << "V1 = V1 * 3.6;" << endl
		<< "V2 = V2 * 3.6;" << endl;
	fout.close();

	std::cout
		<< endl
		<< "*** RESULTS ***" << endl
		<< "Time elapsed: " << dt << endl
		<< "Ordinary cruise-control: " << endl
		<< " Travel time : " << T2 << " sec" << endl
		<< " Fuel outlay : " << truck2.F / ((N - 1) * L_SEG / 1000) * 100 << " l/100km" << endl
		<< "Adaptive cruise-control: " << endl
		<< " Travel time : " << T1 << " sec" << endl
		<< " Fuel outlay : " << truck1.F / ((N - 1) * L_SEG / 1000) * 100 << " l/100km" << endl << endl;
	system("pause");

	delete[] dt;
	delete[] S;
	delete[] H;
	delete[] A;
	delete[] V1;
	delete[] V2;
	delete[] F1;
	delete[] F2;
	delete[] A1;
	delete[] A2;
	delete[] B1;
	delete[] B2;
	delete[] G1;
	delete[] G2;
}

char *GetTimeDifference(SYSTEMTIME &t1, SYSTEMTIME &t2){
	long t1_ms = t1.wDay * MSPERD + t1.wHour * MSPERH + t1.wMinute * MSPERM + t1.wSecond * MSPERS + t1.wMilliseconds,
		   t2_ms = t2.wDay * MSPERD + t2.wHour * MSPERH + t2.wMinute * MSPERM + t2.wSecond * MSPERS + t2.wMilliseconds;
	long delta = t2_ms - t1_ms;

	int d, h, m, s, ms;
	d = delta / MSPERD;
	delta -= d * MSPERD;
	h = delta / MSPERH;
	delta -= h * MSPERH;
	m = delta / MSPERM;
	delta -= m * MSPERM;
	s = delta / MSPERS;
	delta -= s * MSPERS;
	ms = delta;

	char *res = new char[30], buf[10];
	strcpy(res, "");
	if(d != 0){
		itoa(d, buf, 10);
		strcat(res, buf);
		strcat(res, "d, ");
	}
	if(h != 0){
		itoa(h, buf, 10);
		strcat(res, buf);
		strcat(res, "h, ");
	}
	if(m != 0){
		itoa(m, buf, 10);
		strcat(res, buf);
		strcat(res, "m, ");
	}
	if(s != 0){
		itoa(s, buf, 10);
		strcat(res, buf);
		strcat(res, "s, ");
	}
	itoa(ms, buf, 10);
	strcat(res, buf);
	strcat(res, "ms");

	return res;
}