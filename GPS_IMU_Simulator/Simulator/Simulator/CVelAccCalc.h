#include "CMatrix.h"
using namespace System;

ref class CVelAccCalc 
{
private:
		
	CMatrix* carNextVelocity;
	CMatrix* carPreviousVelocity;
public:
	CMatrix* timeV;
	CMatrix* carCurrentVelocity;
	CMatrix* carAcceleration;
	CVelAccCalc();
	~CVelAccCalc();
	void Differentatior(CMatrix& , CMatrix& , double , double , CMatrix& );
	double CarAccelerationCalc(CMatrix*, double);
	void ReboundVelocity(CMatrix* , double );
};