#pragma once

#ifndef DATAIO_16012013
#define DATAIO_16012013

#include <fstream>

using namespace std;

int ReadFromDATFile(ifstream &fin, char *name, double *&dst);
void WriteToMFile(ofstream &fout, const double *val, int len, const char *name);
void WriteToMFile(ofstream &fout, const int *val, int len, const char *name);

#endif // DATAIO_16012013