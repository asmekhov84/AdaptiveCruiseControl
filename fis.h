#pragma once

#ifndef FIS_20022013
#define FIS_20022013

typedef struct{
	int type;
	double *params;
} MF;

typedef struct{
	double range[2];
	int N_mfs;
	MF *mfs;
} VAR;

typedef struct{
	int *subconditions;
	int conclusion;
	double weight;
} RULE;

class CFuzzyInferenceSystem{
private:
	int N_in, // count of input variables
		N_rule; // count of rules
	VAR *var_in, // array of input variables
		var_out; // single output variable 
	RULE *rules; // array of rules
//	double *plot; // values of output variable MFs [var_out.N_mfs x N_PNT]
	void LoadVar(char *fullName, char *app_name, VAR &var);
	double GetMFValue(MF &mf, double x);
public:
	void Load(char *fileName);
	~CFuzzyInferenceSystem();
	double GetValue(double *inputs);
};

#endif //FIS_20022013