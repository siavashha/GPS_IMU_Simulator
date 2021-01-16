#include "CMatrix.h"
using namespace System;

ref class CVelPosCalc 
{
private:

	CMatrix* carCurrentPosition;
	CMatrix* carNextPosition;

public:
	CMatrix* timeX;
	CMatrix* carPreviousPosition;
	CMatrix* carVelocity;
	CVelPosCalc();
	~CVelPosCalc();
	void Integrator(CMatrix& , CMatrix& , double , double , CMatrix& );
	double CarPositionCalc(CMatrix*, double);
	void ReboundPosition(CMatrix*, double);
};