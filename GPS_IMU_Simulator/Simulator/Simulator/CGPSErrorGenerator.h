#include "CMatrix.h"
#include "CDOPCalculation.h"
#include <time.h>
#include <math.h>
#include <stdlib.h>
//#include <stdio.h>

using namespace System;

ref class CGPSErrorGenerator
{
private:

public:
	CMatrix* simulatedGPSPos;
	CGPSErrorGenerator();
	~CGPSErrorGenerator();
	double StandardNormalGenerator();
	//void ErrorGenerator(double, CMatrix* , CMatrix* );
	void ErrorGenerator(int , CMatrix* );

};