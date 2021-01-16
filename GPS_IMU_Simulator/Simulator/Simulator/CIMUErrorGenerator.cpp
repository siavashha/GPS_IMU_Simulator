#include "CIMUErrorGenerator.h"

CIMUErrorGenerator::CIMUErrorGenerator()
{
	srand( (unsigned)time( NULL ) );
	simulatedAccelerometer = new CMatrix(3,1);
	simulatedGyro =  new CMatrix(3,1);
}

CIMUErrorGenerator::~CIMUErrorGenerator()
{	
	delete simulatedAccelerometer;
	delete simulatedGyro;
}

double CIMUErrorGenerator::StandardNormalGenerator()
{
	// generate random number with uniform distribution
	double PI=3.14159265358979323846;
	double var1;
	double var2;
	
	do 
	{
		var1 =  ( double(rand()) / double(RAND_MAX) ) ;
		var2 =  ( double(rand()) / double(RAND_MAX) ) ;
	} while (!(( var1 > 0 && var1 <= 1.0 ) && ( var2 > 0 && var2 <= 1.0 )));

    double R=sqrt( -2 * log(var1));
	double theta = 2  * var2 * PI;

    //        calculate Standard Normal Distribution
    if (R*cos(theta)<1e10)
	return R*cos(theta);
	else if (R*sin(theta)<1e10)
	return R*sin(theta);
	else
	return 0;
}

void CIMUErrorGenerator::ErrorGenerator(double gradeIMU, CMatrix* errorFreeaccelerometer, CMatrix* errorFreegyro)
{
	gradeIMU = 0;
	double AccBias = 0;
	double AccScale = 0;
	double AccSigma = 0;
	double gyroBias = 0;
	double gyroScale = 0;
	double gyroSigma = 0;
	double randVar = 0;
	if ( gradeIMU == 0)
	{
		 AccBias = 0;
		 AccScale = 0;
		 AccSigma = 0;
		 gyroBias=0;
		 gyroScale = 0;
		 gyroSigma = 0;
	}
	else if ( gradeIMU == 1)
	{
		 AccBias = 2e-4;
		 AccScale = 40e-6;
		 AccSigma = 5e-5;
		 gyroBias=4.8e-8;
		 gyroScale = 1e-6;
		 gyroSigma = 2.9e-7;
	}
	else if(gradeIMU== 2)
	{
		 AccBias = 9.8e-3;
		 AccScale = 300e-6;
		 AccSigma = 1.5e-3;
		 gyroBias=4.8e-6;
		 gyroScale = 150e-6;
		 gyroSigma = 3.6e-5;
	}
	else if(gradeIMU== 3)
	{
		 AccBias = 0.01;
		 AccScale = 1e-2;
		 AccSigma = 0.0083;
		 gyroBias= 0.0524;
		 gyroScale = 1e-2;
		 gyroSigma = 3e-5;
	}
	else
	{
		Console::WriteLine("error in IMU type or error generator");
	}

	randVar=StandardNormalGenerator();
	(*simulatedAccelerometer)(0,0)=AccBias+(1+AccScale)*(*errorFreeaccelerometer)(0,0)+AccSigma*randVar;
	randVar=StandardNormalGenerator();
	(*simulatedAccelerometer)(1,0)=AccBias+(1+AccScale)*(*errorFreeaccelerometer)(1,0)+AccSigma*randVar;
	randVar=StandardNormalGenerator();
	(*simulatedAccelerometer)(2,0)=AccBias+(1+AccScale)*(*errorFreeaccelerometer)(2,0)+AccSigma*randVar;

	randVar=StandardNormalGenerator();
	(*simulatedGyro)(0,0)=gyroBias+(1+gyroScale)*(*errorFreegyro)(0,0)+gyroSigma*randVar;
	randVar=StandardNormalGenerator();
	(*simulatedGyro)(1,0)=gyroBias+(1+gyroScale)*(*errorFreegyro)(1,0)+gyroSigma*randVar;
	randVar=StandardNormalGenerator();
	(*simulatedGyro)(2,0)=gyroBias+(1+gyroScale)*(*errorFreegyro)(2,0)+gyroSigma*randVar;

}