#include "CMatrix.h"
//#include "CConversion.h"
#include <math.h>
using namespace System;

ref class CErrorFreeAccelerometer 
{
private:

public:
	CMatrix* errorFreeAccelerometer;
	CErrorFreeAccelerometer();
	~CErrorFreeAccelerometer();
	void ErrorFreeAccelerometerCalc(CMatrix*,CMatrix*,CMatrix*,CMatrix*);
};