#include "CMatrix.h"
#include <math.h>
using namespace System;

ref class CEstimatedErrorCorrection 
{
private:
	
public:
	CMatrix* updatedRb2n;
	CMatrix* updatedVelocityInn;
	CMatrix* updatedGeodeticPositionInn;
	CMatrix* updatedBiasAccelerometer;
	CMatrix* updatedBiasGyro;
	CMatrix* updatedScaleFactorErrorAccelerometer;
	//CMatrix* updatedScaleFactorErrorGyro;
	CEstimatedErrorCorrection();
	~CEstimatedErrorCorrection();
	void ErrorCorrection(CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* );
};