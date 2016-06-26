#pragma once

#ifndef ROAD_28032013
#define ROAD_28032013

class CRoadProfile{
public:
	int N; // number of road nodes
	double *S; // coord. of each node
	double *H; // height in each node
	double *A; // angle in each node
public:
	CRoadProfile(char *profileName);
	~CRoadProfile();
	double GetH(double _S);
	double GetA(double _S);
};

#endif //ROAD_28032013