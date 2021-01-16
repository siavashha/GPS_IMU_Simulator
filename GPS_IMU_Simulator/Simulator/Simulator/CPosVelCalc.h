#include "CMatrix.h"
using namespace System;

ref class CPosVelCalc 
{
private:

	CMatrix* carCurrentPosition;
	CMatrix* carNextPosition;

public:
	CMatrix* timeX;
	CMatrix* carPreviousPosition;
	CMatrix* carVelocity;
	CPosVelCalc();
	~CPosVelCalc();
	void Differentatior(CMatrix& , CMatrix& , double , double , CMatrix& );
	double CarVelocityCalc(CMatrix*, double);
	void ReboundPosition(CMatrix*, double);
};