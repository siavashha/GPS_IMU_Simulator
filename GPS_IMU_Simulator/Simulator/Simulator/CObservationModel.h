#include "CMatrix.h"
#include <math.h>
using namespace System;

ref class CObservationModel
{
private:

public:
	CMatrix* designMatrixH;
	CMatrix* observationNoiseModelR;
	CObservationModel();
	~CObservationModel();
	void ObservationModelCalc(int);
	void GetR( int );
};