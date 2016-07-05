#pragma once

#ifndef DATAIO_16012013
#define DATAIO_16012013

#include <stdio.h>

int ReadFromDATFile(FILE *fin, char *name, double *&dst);

#endif // DATAIO_16012013