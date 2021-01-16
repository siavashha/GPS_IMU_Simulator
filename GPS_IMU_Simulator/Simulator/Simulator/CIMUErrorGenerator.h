#include "CMatrix.h"
//#include "CConversion.h"
#include <time.h>
#include <math.h>
#include <stdlib.h>
//#include <stdio.h>

using namespace System;

ref class CIMUErrorGenerator
{
private:

public:
	CMatrix* simulatedAccelerometer;
	CMatrix* simulatedGyro;
	CIMUErrorGenerator();
	~CIMUErrorGenerator();
	double StandardNormalGenerator();
	//void ErrorGenerator(double, CMatrix* , CMatrix* );
	void ErrorGenerator(double , CMatrix* , CMatrix* );

};