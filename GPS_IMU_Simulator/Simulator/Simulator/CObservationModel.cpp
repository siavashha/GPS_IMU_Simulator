#include "CObservationModel.h"
//#include "CDisplayMatrixElements.h"

CObservationModel::CObservationModel()
{
	designMatrixH = new CMatrix(2,16);
	observationNoiseModelR = new CMatrix(2,2);
}

CObservationModel::~CObservationModel()
{
	delete designMatrixH;
	delete observationNoiseModelR;
}

void CObservationModel::ObservationModelCalc(int gradeGPS)
{
	(*designMatrixH)(0,5)=1;
	(*designMatrixH)(1,6)=1;
	//(*designMatrixH)(2,8)=1;
	GetR(gradeGPS);
}

void CObservationModel::GetR( int gradeGPS)
{
	double posAccuracyInMeter;
	if (gradeGPS==0)
	{
		posAccuracyInMeter = 0.01;  
	}
	if (gradeGPS==1)
	{
		posAccuracyInMeter = 0.01;  
	}
	if (gradeGPS==2)
	{
		posAccuracyInMeter = 1;  
	}
	if (gradeGPS==3)
	{
		posAccuracyInMeter = 8;  
	}
	(*observationNoiseModelR)(0,0) = posAccuracyInMeter/6400000 * posAccuracyInMeter/6400000;
	(*observationNoiseModelR)(1,1) = posAccuracyInMeter/6400000 * posAccuracyInMeter/6400000;
}