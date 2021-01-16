#include "COrientationCalc.h"

COrientationCalc::COrientationCalc()
{
	carPitchRollYaw = new CMatrix(3,1);
}

COrientationCalc::~COrientationCalc()
{
	delete carPitchRollYaw;
}

double COrientationCalc::Degree2Rad(double angleInDeg)
{
	double PI=3.14159265358979323846;
	double angleInRad=angleInDeg*PI/180.0;
	return angleInRad;
}

double COrientationCalc::Radian2Deg(double angleInRad)
{
	double PI=3.14159265358979323846;
	double angleInDeg=angleInRad*180.0/PI;
	return angleInDeg;
}

void COrientationCalc::CarOrientationCalc(CMatrix* frontBumper , CMatrix* rearBumper)
{
	double dx;
	double dy;
	double dz;
    dx = (*frontBumper)(0,0)-(*rearBumper)(0,0);
    dy = (*frontBumper)(1,0)-(*rearBumper)(1,0);
    dz = (*frontBumper)(2,0)-(*rearBumper)(2,0);
	
	double carLength=sqrt(dx*dx+dy*dy+dz*dz);
	double azimuth;
	double pitch;
	double roll;

	azimuth=atan2(dx,dy);
	pitch= asin(dz/carLength);
	roll= 0;
	(*carPitchRollYaw)(0,0)=pitch;
	(*carPitchRollYaw)(1,0)=roll;
	(*carPitchRollYaw)(2,0)=azimuth;
}

void COrientationCalc::CarDirectCosineMatrixCalc(CMatrix* carPRY, CMatrix* carDCMMatrix_body2NED)
{
	double pitch = (*carPRY)(0,0);
	double roll = (*carPRY)(1,0);
	double azimuth = (*carPRY)(2,0);

	//CMatrix* carDCMMatrix_body2NED = new CMatrix(3,3);
	(*carDCMMatrix_body2NED)(0,0)=cos(-azimuth)*cos(-pitch);
	(*carDCMMatrix_body2NED)(0,1)=cos(-azimuth)*sin(-pitch)*sin(-roll)+sin(-azimuth)*cos(-roll);
	(*carDCMMatrix_body2NED)(0,2)=-cos(-azimuth)*sin(-pitch)*cos(-roll)+sin(-azimuth)*sin(-roll);

	(*carDCMMatrix_body2NED)(1,0)=-sin(-azimuth)*cos(-pitch);
	(*carDCMMatrix_body2NED)(1,1)=-sin(-azimuth)*sin(-pitch)*sin(-roll)+cos(-azimuth)*cos(-roll);
	(*carDCMMatrix_body2NED)(1,2)=sin(-azimuth)*sin(-pitch)*cos(-roll)+cos(-azimuth)*sin(-roll);

	(*carDCMMatrix_body2NED)(2,0)=sin(-pitch);
	(*carDCMMatrix_body2NED)(2,1)=-cos(-pitch)*sin(-roll);
	(*carDCMMatrix_body2NED)(2,2)=cos(-pitch)*cos(-roll);

	//CMatrix* carDCMMatrix_body2ENU = new CMatrix(3,3);
	//(*carDCMMatrix_body2NED)(0,0) = -sin(-azimuth)*sin(-pitch)*sin(-roll)+cos(-azimuth)*cos(-roll);
	//(*carDCMMatrix_body2NED)(0,1) = -sin(-azimuth)*cos(-pitch);
	//(*carDCMMatrix_body2NED)(0,2) = -(sin(-azimuth)*sin(-pitch)*cos(-roll)+cos(-azimuth)*sin(-roll));

	//(*carDCMMatrix_body2NED)(1,0) = cos(-azimuth)*sin(-pitch)*sin(-roll)+sin(-azimuth)*cos(-roll);
	//(*carDCMMatrix_body2NED)(1,1) = cos(-azimuth)*cos(-pitch);
	//(*carDCMMatrix_body2NED)(1,2) = -(-cos(-azimuth)*sin(-pitch)*cos(-roll)+sin(-azimuth)*sin(-roll));

	//(*carDCMMatrix_body2NED)(2,0) = (-(-cos(-pitch)*sin(-roll)));
	//(*carDCMMatrix_body2NED)(2,1) = ((-sin(-pitch)));
	//(*carDCMMatrix_body2NED)(2,2) = (cos(-pitch)*cos(-roll));
	
	//Console::WriteLine("///////////////");
	//Console::WriteLine((*carDCMMatrix_body2NED)(0,0));
	//Console::WriteLine((*carDCMMatrix_body2NED)(1,0));
	//Console::WriteLine((*carDCMMatrix_body2NED)(1,1));
	//Console::WriteLine((*carDCMMatrix_body2NED)(2,1));

}