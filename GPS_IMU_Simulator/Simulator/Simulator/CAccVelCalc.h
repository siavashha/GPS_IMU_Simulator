#include "CMatrix.h"
using namespace System;

ref class CAccVelCalc 
{
private:
		
	CMatrix* carNextVelocity;
	CMatrix* carPreviousVelocity;
public:
	CMatrix* timeV;
	CMatrix* carCurrentVelocity;
	CMatrix* carAcceleration;
	CAccVelCalc();
	~CAccVelCalc();
	void Integrator(CMatrix& , CMatrix& , double , double , CMatrix& );
	double CarVelocityCalc(CMatrix*, double);
	void ReboundVelocity(CMatrix* , double );
};