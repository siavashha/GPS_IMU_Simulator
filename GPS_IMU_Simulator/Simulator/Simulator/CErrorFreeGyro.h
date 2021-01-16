#include "CMatrix.h"
//#include "CConversion.h"
#include <math.h>
using namespace System;

ref class CErrorFreeGyro
{
private:

public:
	CMatrix* errorFreeGyro;
	CErrorFreeGyro();
	~CErrorFreeGyro();
	void ErrorFreeGyroCalc(CMatrix*,CMatrix*,CMatrix*,CMatrix*);
};