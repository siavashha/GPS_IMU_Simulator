#include "CMatrix.h"
using namespace System;

ref class CAngularRateCalc 
{
private:

	CMatrix* carCurrentEulerAngle;
	CMatrix* carNextEulerAngles;
		
	CMatrix* carPreviousAngularRate;
	CMatrix* carNextAngularRate;
	CMatrix* carAngularRate;

public:

	CMatrix* timeT;
	CMatrix* timeW;
	CMatrix* carPreviousEulerAngle;
	CMatrix* carCurrentAngularRate;
	
	CAngularRateCalc();
	~CAngularRateCalc();
	void Differentatior(CMatrix& , CMatrix& , double , double , CMatrix& );
	double CarAngularRateCalc(CMatrix*  , double);
	void ReboundOrientation(CMatrix*, double);
	void ReboundAngularRate(CMatrix*, double);

};