#include "CMatrix.h"
#include "CStateModel.h"
#include "CObservationModel.h"
#include <math.h>

using namespace std;
#include "CMatrix.h"
#include <iostream>
#include <fstream>
using namespace System::IO;
using namespace System;

ref class CKalmanFilter 
{
private:
	int gradeIMU;
	int gradeGPS;
	CStateModel^ timeModelUpdate;
	CObservationModel^ observationModelUpdate;
	StreamWriter^ dout;
public:
	CMatrix* x;
	CMatrix* P;
	CKalmanFilter();
	~CKalmanFilter();
	void TimeUpdate(CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* , int , double);
	void ObservationUpdate(int, CMatrix*, CMatrix*);
	double Degree2Rad(double);
};