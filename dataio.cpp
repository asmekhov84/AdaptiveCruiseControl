#include "dataio.h"
#include "common.h"

int ReadFromDATFile(ifstream &fin, char *name, double *&dst){
	// go to begin of the 'name' data block
	char tmp[BUFSZ], buf[BUFSZ];
	strcpy_s(tmp, "[");
	strcat_s(tmp, name);
	strcat_s(tmp, "]");
	int len = strlen(tmp);
	fin.seekg(0);
	while(!fin.eof()){
		fin >> buf;
		if(!strncmp(buf, tmp, len)) break;
	}
	const int startPos(fin.tellg());

	// count the number of elements in the data block
	char *endPos;
	int cnt(0);
	double src[BUFSZ];
	while(!fin.eof()){
		fin >> buf;
		src[cnt] = strtod(buf, &endPos);
		if(*endPos != NULL) break;
		cnt++;
	}
	dst = new double[cnt];
	memcpy(dst, src, sizeof(double) * cnt);

	return cnt;
}

void WriteToMFile(ofstream &fout, const double *val, int len, const char *name){
	fout << name << " = [";
	for(int i(0); i < len; i++) fout << val[i] << " ";
	fout << "];\n";
}

void WriteToMFile(ofstream &fout, const int *val, int len, const char *name){
	fout << name << " = [";
	for(int i(0); i < len; i++)	fout << val[i] << " ";
	fout << "];\n";
}