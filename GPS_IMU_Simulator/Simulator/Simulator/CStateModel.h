#include "CMatrix.h"
#include <math.h>
using namespace System;

ref class CStateModel
{
private:

public:
	CMatrix* dynamicModelF;
	CMatrix* systemNoiseModelG;
	CMatrix* systemCovarianceMatrixQ;
	CStateModel();
	~CStateModel();
	void StateModelCalc(CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* , int );
	void GetQ( int );
};