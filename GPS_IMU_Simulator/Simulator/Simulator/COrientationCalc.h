#include "CMatrix.h"
#include <math.h>
using namespace System;

ref class COrientationCalc
{
private:

public:

	CMatrix* carPitchRollYaw;
	COrientationCalc();
	~COrientationCalc();
	void CarOrientationCalc(CMatrix* , CMatrix*);
	double Degree2Rad(double );
	double Radian2Deg(double);
	void CarDirectCosineMatrixCalc(CMatrix* ,CMatrix* );
};