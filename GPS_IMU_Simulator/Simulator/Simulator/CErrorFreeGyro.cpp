#include "CErrorFreeGyro.h"
#include "CConversion.h"
#include "COrientationCalc.h"


CErrorFreeGyro::CErrorFreeGyro()
{
	errorFreeGyro = new CMatrix(3,1);
}

CErrorFreeGyro::~CErrorFreeGyro()
{	
	delete errorFreeGyro;
}

void CErrorFreeGyro::ErrorFreeGyroCalc(CMatrix* carPositionGeodeticCoord, CMatrix* carVelocity , CMatrix* carEulerAngle , CMatrix* carAngularRate)
{
	const double w_e= 7.2921150*0.00001;//(Earth Rotation Rate) radian per second
	CConversion^ conversion = gcnew CConversion();

	double M=conversion->PrimeMeridian();
	double N=conversion->PrimeVertical();
	double velE=(*carVelocity)(0,0);
	double velN=(*carVelocity)(1,0);
	double velD=-(*carVelocity)(2,0);

	double lambda=conversion->Degree2Rad((*carPositionGeodeticCoord)(0,0));
	double phi=conversion->Degree2Rad((*carPositionGeodeticCoord)(1,0));
	double height=(*carPositionGeodeticCoord)(2,0);

	double pitch = (*carEulerAngle)(0,0);
	double roll = (*carEulerAngle)(1,0);
	double azimuth = (*carEulerAngle)(2,0);

	double pitchDot = (*carAngularRate)(0,0);
	double rollDot = (*carAngularRate)(1,0);
	double azimuthDot = (*carAngularRate)(2,0);

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.103
	double phiDot=velN/(M+height);
	double lambdaDot = velE/((N+height)*cos(phi));
	double hDot = - velD;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 26 eq. 1.94
	CMatrix* wnb_b = new CMatrix(3,1);
	(*wnb_b)(0,0) = rollDot - azimuthDot * sin(pitch);
	(*wnb_b)(1,0) = pitchDot * cos(roll) + cos(pitch) * sin(roll) * azimuthDot;
	(*wnb_b)(2,0) = - pitchDot * sin(roll) + cos(pitch) * cos(roll) * azimuthDot;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 25 eq. 1.89
	CMatrix* win_n = new CMatrix(3,1);
	(*win_n)(0,0) = (lambdaDot + w_e) * cos(phi);
	(*win_n)(1,0) = - phiDot;
	(*win_n)(2,0) = - (lambdaDot + w_e) * sin(phi);

	CMatrix* wib_b = new CMatrix(3,1);
	CMatrix* wib_n = new CMatrix(3,1);
	CMatrix* Cb_n = new CMatrix(3,3);
	CMatrix* Cn_b = new CMatrix(3,3);
	
	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 130 eq. 4.105
	COrientationCalc^ orientaion = gcnew COrientationCalc();
	orientaion->CarDirectCosineMatrixCalc(carEulerAngle,Cb_n);
			
	/*Console::WriteLine("IMU Simulation");
	Console::Write((*Cb_n)(0,0)+ " "); Console::Write((*Cb_n)(0,1)+ " ");  Console::WriteLine((*Cb_n)(0,2)+ " "); 
	Console::Write((*Cb_n)(1,0)+ " "); Console::Write((*Cb_n)(1,1)+ " ");  Console::WriteLine((*Cb_n)(1,2)+ " "); 
	Console::Write((*Cb_n)(2,0)+ " "); Console::Write((*Cb_n)(2,1)+ " ");  Console::WriteLine((*Cb_n)(2,2)+ " "); */
	
	(*Cn_b) = Cb_n->GetTranspose();
	(*wib_n) = (*Cn_b) * (*win_n);
	(*wib_b) = (*wnb_b) + (*wib_n);
	(*this->errorFreeGyro)=(*wib_b);
	
	delete wnb_b;
	delete win_n;
	delete wib_b;
	delete wib_n;
	delete Cb_n;
	delete Cn_b;

	delete conversion;
	delete orientaion;
}

