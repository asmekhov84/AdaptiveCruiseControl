#include "dataio.h"
#include "common.h"
#include <string.h>
#include <stdlib.h>

int ReadFromDATFile(FILE *fin, char *name, double *&dst){
	// go to begin of the 'name' data block
	char tmp[BUFSZ], buf[BUFSZ];
	strcpy_s(tmp, "[");
	strcat_s(tmp, name);
	strcat_s(tmp, "]");
	int len = strlen(tmp);
	fseek(fin, 0, SEEK_SET);	
	while(!feof(fin)){
		fscanf(fin, "%s/n", buf);
		if(!strncmp(buf, tmp, len)) break;
	}
	const int startPos(ftell(fin));

	// count the number of elements in the data block
	char *endPos;
	int cnt(0);
	double src[BUFSZ];
	while(!feof(fin)){
		fscanf(fin, "%s/n", buf);
		src[cnt] = strtod(buf, &endPos);
		if(*endPos != NULL) break;
		cnt++;
	}
	dst = new double[cnt];
	memcpy(dst, src, sizeof(double) * cnt);

	return cnt;
}