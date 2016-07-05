#include "fis.h"
#include "common.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

#define MF_MAX_NAME_LEN 10
#define MF_CNT 3
const char MF_TYPE[MF_CNT][MF_MAX_NAME_LEN] = {"trimf", "trapmf", "constant"};

void ReadFromFISFile(char *fileName, char *section, char *buf);

CFuzzyInferenceSystem::~CFuzzyInferenceSystem(){
	// delete input variables
	for(int i(0); i < N_in; i++){
		for(int j(0); j < var_in[i].N_mfs; j++){
			delete[] var_in[i].mfs[j].params;
		}
		delete[] var_in[i].mfs;
	}
	delete[] var_in;
	// delete output variable
	for(int j(0); j < var_out.N_mfs; j++){
		delete[] var_out.mfs[j].params;
	}
	delete[] var_out.mfs;
	// delete rulebase
	for(int i(0); i < N_rule; i++){
		delete[] rules[i].subconditions;
	}
	delete[] rules;
}

void CFuzzyInferenceSystem::Load(char *fileName){
	char buf1[BUFSZ], *buf2(new char[BUFSZ]), *pos1, fullName[BUFSZ];
	GetCurrentDirectoryA(BUFSZ, fullName);
	strcat_s(fullName, BUFSZ, "\\");
	strcat_s(fullName, BUFSZ, fileName);
	// system
	GetPrivateProfileStringA("System", "NumInputs", 0, buf1, BUFSZ, fullName);
	N_in = atoi(buf1);
	GetPrivateProfileStringA("System", "NumRules", 0, buf1, BUFSZ, fullName);
	N_rule = atoi(buf1);
	// input variables
	var_in = new VAR[N_in];
	for(int i(0); i < N_in; i++){
		strcpy_s(buf1, BUFSZ, "Input");
		_itoa(i + 1, buf1 + 5, 10);
		LoadVar(fullName, buf1, var_in[i]);
	}
	// output variable
	LoadVar(fullName, "Output1", var_out);
	// rules
	rules = new RULE[N_rule];
	memset(buf1, 0, BUFSZ);
	ReadFromFISFile(fileName, "Rules", buf1);
	pos1 = buf1;
	for(int i(0); i < N_rule; i++){
		rules[i].subconditions = new int[N_in];
		for(int j(0); j < N_in; j++){
			rules[i].subconditions[j] = atoi(pos1);
			pos1 = strchr(pos1, ' ') + 1;
		}
		rules[i].conclusion = atoi(pos1);
		pos1 = strchr(pos1, '(') + 1;
		rules[i].weight = atof(pos1);
		pos1 = strchr(pos1, '\n') + 1;
	}
}

void CFuzzyInferenceSystem::LoadVar(char *fullName, char *app_name, VAR &var){
	char buf1[BUFSZ], *pos1, *pos2;
	int params_cnt, i, j;
	// range
	GetPrivateProfileStringA(app_name, "Range", 0, buf1, BUFSZ, fullName);
	var.range[0] = atof(buf1 + 1);
	var.range[1] = atof(strchr(buf1, ' ') + 1);
	// membership functions
	GetPrivateProfileStringA(app_name, "NumMFs", 0, buf1, BUFSZ, fullName);
	var.N_mfs = atoi(buf1);
	var.mfs = new MF[var.N_mfs];
	for(i = 0; i < var.N_mfs; i++){
		strcpy_s(buf1, BUFSZ, "MF");
		_itoa(i + 1, buf1 + 2, 10);
		// define MF's type
		GetPrivateProfileStringA(app_name, buf1, 0, buf1, BUFSZ, fullName);
		pos1 = strchr(buf1 + 1, '\'') + 3;
		pos2 = strchr(pos1, '\'');
		for(j = 0; j < MF_CNT; j++){
			if(strncmp(pos1, MF_TYPE[j], (pos2 - pos1) / sizeof(char)) == 0) break;
		}
		var.mfs[i].type = j;
		// load MF's parameters
		switch(var.mfs[i].type){
			case(0):
				params_cnt = 3;
				break;
			case(1):
				params_cnt = 4;
				break;
			case(2):
				params_cnt = 1;
				break;
		}
		var.mfs[i].params = new double[params_cnt];
		pos2 += 3;
		for(j = 0; j < params_cnt; j++){
			var.mfs[i].params[j] = atof(pos2);
			pos2 = strchr(pos2, ' ') + 1;
		}
	}
}

double CFuzzyInferenceSystem::GetValue(double *inputs){
	int i, j;
	double x, y, p, num(0), denum(0);
	for(i = 0; i < N_rule; i++){
		y = 1;
		for(j = 0; j < N_in; j++){
			// fuzzification
			p = GetMFValue(var_in[j].mfs[rules[i].subconditions[j] - 1], inputs[j]);
			// aggregation (AndMethod='prod')
			y *= p;
		}
		y *= rules[i].weight;
		// activation (ImpMethod='prod')
		x = GetMFValue(var_out.mfs[rules[i].conclusion - 1], 0);
		// accumulation (AggMethod='sum')
		num += x * y;
		denum += y;
	}
	// defuzzification (DefuzzMethod='wtaver')
	return num / denum;
}

double CFuzzyInferenceSystem::GetMFValue(MF &mf, double x){
	double y1, y2, res;
	switch(mf.type){
		case(0):
			y1 = (x - mf.params[0]) / (mf.params[1] - mf.params[0]);
			y2 = (mf.params[1] - x) / (mf.params[2] - mf.params[1]) + 1;
			res = max(min(y1, y2), 0);
			break;
		case(1):
			y1 = (x - mf.params[0]) / (mf.params[1] - mf.params[0]);
			y2 = (mf.params[2] - x) / (mf.params[3] - mf.params[2]) + 1;
			res = min(max(min(y1, y2), 0), 1);
			break;
		case(2):
			res = mf.params[0];
			break;
	}
	return res;
}

void ReadFromFISFile(char *fileName, char *section, char *buf){
	FILE *fin = fopen(fileName, "r");
	char tmp[BUFSZ], str[BUFSZ];
	strcpy_s(str, BUFSZ, "[");
	strcat_s(str, BUFSZ, section);
	strcat_s(str, BUFSZ, "]");
	while(strncmp(tmp, str, strlen(str)) != 0){
		fgets(tmp, BUFSZ, fin);
	}
	char c;
	int i(0);
	memset(tmp, 0, BUFSZ);
	while ((buf[i] = fgetc(fin)) > 0)
	{
		i++;
	}
	buf[i - 1] = '\0';
	fclose(fin);
}