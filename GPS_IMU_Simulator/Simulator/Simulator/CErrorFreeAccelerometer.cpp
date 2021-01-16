#include "CErrorFreeAccelerometer.h"
#include "CConversion.h"
#include "COrientationCalc.h"

CErrorFreeAccelerometer::CErrorFreeAccelerometer()
{
	errorFreeAccelerometer = new CMatrix(3,1);
}

CErrorFreeAccelerometer::~CErrorFreeAccelerometer()
{	
	delete errorFreeAccelerometer;
}

void CErrorFreeAccelerometer::ErrorFreeAccelerometerCalc(CMatrix* carPositionGeodeticCoord , CMatrix* carVelocity , CMatrix* carAcceleration, CMatrix* carEulerAngle)
{
	CConversion^ conversion = gcnew CConversion();
	double M=conversion->PrimeMeridian();
	double N=conversion->PrimeVertical();
	double velE = (*carVelocity)(0,0);
	double velN = (*carVelocity)(1,0);
	double velD = -(*carVelocity)(2,0);

	double accE = (*carAcceleration)(0,0);
	double accN = (*carAcceleration)(1,0);
	double accD = -(*carAcceleration)(2,0);

	//Console::WriteLine("IMU Simulation");
	//Console::Write(velE+ " "); Console::Write(velN+ " ");  Console::WriteLine(-velD+ " "); 
	//Console::WriteLine();

		/*	
	Console::WriteLine("IMU Simulation");
	Console::Write((*Cb_n)(0,0)+ " "); Console::Write((*Cb_n)(0,1)+ " ");  Console::WriteLine((*Cb_n)(0,2)+ " "); 
	Console::Write((*Cb_n)(1,0)+ " "); Console::Write((*Cb_n)(1,1)+ " ");  Console::WriteLine((*Cb_n)(1,2)+ " "); 
	Console::Write((*Cb_n)(2,0)+ " "); Console::Write((*Cb_n)(2,1)+ " ");  Console::WriteLine((*Cb_n)(2,2)+ " "); 
	*/
	const double w_e = 7.2921150*0.00001;//(Earth Rotation Rate) radian per second
	
	double lambda = conversion->Degree2Rad((*carPositionGeodeticCoord)(0,0));
	double phi = conversion->Degree2Rad((*carPositionGeodeticCoord)(1,0));
	double height = (*carPositionGeodeticCoord)(2,0);

	//gravity at this latitude and height
    double g0 = 9.7803267714*((1+0.001931851318513863*sin(phi)*sin(phi))/(sqrt(1-0.00669437999013*sin(phi)*sin(phi))));
	//free air anamoly
	double g = g0-0.000003086*height;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.103
	double phiDot = velN / (M + height);
	double lambdaDot = velE / ((N + height) * cos(phi));
	double hDot = -velD;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.102
	double aN = accN ;//- (-2 * w_e * sin(phi) * velE + phiDot * velD - lambdaDot * sin(phi) * velE);
	double aE = accE ;//- (2 * w_e * sin(phi) * velN + 2 * w_e * cos(phi) * velD - lambdaDot * sin(phi) * velN + lambdaDot*cos(phi)*velD);
	double aD = accD ;//- (g - 2 * w_e * cos(phi) * velE - lambdaDot * cos(phi) * velE - phiDot * velN);
	
	CMatrix* Cb_n = new CMatrix(3,3);
	CMatrix* sensedAccelerationInNED = new CMatrix(3,1);
	(*sensedAccelerationInNED)(0,0) = aN;
	(*sensedAccelerationInNED)(1,0) = aE;
	(*sensedAccelerationInNED)(2,0) = aD;

	//Console::WriteLine("prior acceleration");
	//Console::WriteLine(aN);
	//Console::WriteLine(aE);
	//Console::WriteLine(aD);

	COrientationCalc^ orientaion = gcnew COrientationCalc();
	orientaion->CarDirectCosineMatrixCalc(carEulerAngle,Cb_n);
	CMatrix* Cn_b = new CMatrix(3,3);
	(*Cn_b) = Cb_n->GetTranspose();
	(*this->errorFreeAccelerometer)=(*Cn_b)*(*sensedAccelerationInNED);//
	
	//Console::WriteLine((*sensedAccelerationInNED)(1,0));
	//Console::WriteLine(((*this->accelerometerOutput))(1,0));
	delete Cn_b;
	delete Cb_n;
	delete sensedAccelerationInNED;
}

