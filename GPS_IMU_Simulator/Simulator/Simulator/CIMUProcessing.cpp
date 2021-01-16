#include "CIMUProcessing.h"
#include "CConversion.h"
#include "COrientationCalc.h"


CIMUProcessing::CIMUProcessing()
{
	velocityInn = new CMatrix(3,1);
	Rb2n = new CMatrix(3,3);
	positionInn = new CMatrix(3,1);
	biasAcc = new CMatrix(3,1);
	biasGyro = new CMatrix(3,1);
	scaleFactorErrorAcc = new CMatrix(3,1);
	previousAccelerationInnENU= new CMatrix(3,1);
	//scaleFactorErrorGyro = new CMatrix(3,1);
	CDisplayMatrixElements^ ssss =  gcnew CDisplayMatrixElements();
}

CIMUProcessing::~CIMUProcessing()
{	
	delete Rb2n;
	delete positionInn;
	delete velocityInn;
	delete biasAcc;
	delete biasGyro;
	delete scaleFactorErrorAcc;
	delete previousAccelerationInnENU;
	//delete scaleFactorErrorGyro;
}

void CIMUProcessing::PositionCalc(CMatrix* accelerometerMeasurement , CMatrix* gyroMeasurement , double interval )
{
	CMatrix* sensedAccelerationInb = new CMatrix(3,1);
	CMatrix* sensedAngRatei2bInb = new CMatrix(3,1);
	(*sensedAccelerationInb)(0,0) = (*accelerometerMeasurement)(0,0);
	(*sensedAccelerationInb)(1,0) = (*accelerometerMeasurement)(1,0);
	(*sensedAccelerationInb)(2,0) = (*accelerometerMeasurement)(2,0);
	(*sensedAngRatei2bInb)(0,0) = (*gyroMeasurement)(0,0);
	(*sensedAngRatei2bInb)(1,0) = (*gyroMeasurement)(1,0);
	(*sensedAngRatei2bInb)(2,0) = (*gyroMeasurement)(2,0);

	//IMUErrorRemoval(accelerometerMeasurement , gyroMeasurement);
	
	const double w_e= 7.2921150*0.00001;//(Earth Rotation Rate) radian per second
	CConversion^ conversion = gcnew CConversion();
	double M=conversion->PrimeMeridian();
	double N=conversion->PrimeVertical();

	double velE=(*velocityInn)(0,0);
	double velN=(*velocityInn)(1,0);
	double velD=-(*velocityInn)(2,0);
		
	double lambda=conversion->Degree2Rad((*positionInn)(0,0));
	double phi=conversion->Degree2Rad((*positionInn)(1,0));
	double height=(*positionInn)(2,0);

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.103
	double phiDot=velN/(M+height);
	double lambdaDot=velE/((N+height)*cos(phi));
	double hDot=-velD;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 25 eq. 1.89
	CMatrix* win_n = new CMatrix(3,1);
	(*win_n)(0,0) = (lambdaDot + w_e) * cos(phi);
	(*win_n)(1,0) = - phiDot;
	(*win_n)(2,0) = - (lambdaDot + w_e) * sin(phi);
	double w_bx = (*sensedAngRatei2bInb)(0,0);
	double w_by = (*sensedAngRatei2bInb)(1,0);
	double w_bz = (*sensedAngRatei2bInb)(2,0);
	
	double Barray [3*3] = { 0 , -w_bz * interval , w_by * interval , w_bz * interval , 0 , -w_bx * interval , -w_by * interval , w_bx * interval , 0};
	
	CMatrix* B = new CMatrix(Barray , 3 , 3);
	double s = sqrt(w_bx * w_bx + w_by * w_by + w_bz * w_bz) * interval;
	CMatrix* eyeMatrix = new CMatrix(3,3);
	eyeMatrix->SetIdentity(3);
	(*Rb2n) = (*Rb2n) * ((*eyeMatrix) + (*B) * (sin(s) / s)  + (*B) * (*B) * ( (1 - cos(s)) / (s * s) ) );
	delete B;
	delete eyeMatrix;

	////Quaternions
	//CMatrix* Rn2b = new CMatrix(3,1);
	//(*Rn2b) = Rb2n->GetTranspose();

	//CMatrix* delAngn2bInb = new CMatrix(3,1);////Angular velocity measurements(i2b)
	//(*delAngn2bInb) = ( (*sensedAngRatei2bInb) - (*Rn2b) * (*win_n) ) * interval;

	//CMatrix* quaternion = new CMatrix(4,1);//Quaternion vector
	//R2Quaternion(*Rb2n, *quaternion);

	//CMatrix* updateQuaternion = new CMatrix(4,1);//Quaternion vector
	//UpdateQuaternion( *delAngn2bInb, *quaternion , *updateQuaternion);
	//	
	////Using quaternion calculate the new b2e rotation matrix
	//CMatrix* updateRb2n = new CMatrix(3,3);
	//Quaternion2R(((*updateQuaternion)+(*quaternion))/2 , *updateRb2n);

	//delete Rn2b;
	//delete delAngn2bInb;
	//delete quaternion;
	//delete updateQuaternion;
	//delete updateRb2n;

	//Console::WriteLine("//////testRb2n///////");
	//Console::WriteLine( (*testRb2n)(0,0) + "   " +(*testRb2n)(0,1) + "   " + (*testRb2n)(0,2) );
	//Console::WriteLine(	(*testRb2n)(1,0) + "   " +(*testRb2n)(1,1) + "   " + (*testRb2n)(1,2) );
	//Console::WriteLine(	(*testRb2n)(2,0) + "   " +(*testRb2n)(2,1) + "   " + (*testRb2n)(2,2) );

	//Console::WriteLine("//////updateRb2n///////");
	//Console::WriteLine( (*updateRb2n)(0,0) + "   " +(*updateRb2n)(0,1) + "   " + (*updateRb2n)(0,2) );
	//Console::WriteLine(	(*updateRb2n)(1,0) + "   " +(*updateRb2n)(1,1) + "   " + (*updateRb2n)(1,2) );
	//Console::WriteLine(	(*updateRb2n)(2,0) + "   " +(*updateRb2n)(2,1) + "   " + (*updateRb2n)(2,2) );

	//Console::WriteLine("//////Rb2n///////");
	//Console::WriteLine( (*Rb2n)(0,0) + "   " +(*Rb2n)(0,1) + "   " + (*Rb2n)(0,2) );
	//Console::WriteLine(	(*Rb2n)(1,0) + "   " +(*Rb2n)(1,1) + "   " + (*Rb2n)(1,2) );
	//Console::WriteLine(	(*Rb2n)(2,0) + "   " +(*Rb2n)(2,1) + "   " + (*Rb2n)(2,2) );

	//Console::WriteLine();
	//Console::WriteLine();
	
	//(*Rb2n) = (*testRb2n);
	/**********************************************/
	//Calcuation of position using accelerometer

	//calculate acceleration in e
	CMatrix* sensedAccelerationInn = new CMatrix(3,1);
	(*sensedAccelerationInn) = (*Rb2n)* (*sensedAccelerationInb);
	
	double aN = (*sensedAccelerationInn)(0,0);
	double aE = (*sensedAccelerationInn)(1,0);
	double aD = (*sensedAccelerationInn)(2,0);

	delete sensedAccelerationInn;
	//gravity at this latitude and height
    double g0 = 9.7803267714*((1+0.001931851318513863*sin(phi)*sin(phi))/(sqrt(1-0.00669437999013*sin(phi)*sin(phi))));
	//free air anamoly
	double g = g0-0.000003086*height;

	//Jekeli (2010), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.102
	double accN = aN ;//+ (-2 * w_e * sin(phi) * velE + phiDot * velD - lambdaDot * sin(phi) * velE);
	double accE = aE ;//+ (2 * w_e * sin(phi) * velN + 2 * w_e * cos(phi) * velD - lambdaDot * sin(phi) * velN + lambdaDot * cos(phi) * velD);
	double accD = aD ;//+ (g - 2 * w_e * cos(phi) * velE - lambdaDot * cos(phi) * velE - phiDot * velN);
	
	double updateaccelerationENUIneArray [3] = {  accE , accN ,- accD};
	CMatrix* updateAccelerationInnENU = new CMatrix(updateaccelerationENUIneArray,3,1);
		
	CMatrix* updateVelocityInn = new CMatrix(3,1);
	(*updateVelocityInn) = (*updateAccelerationInnENU ) * ( interval) + (*velocityInn) ;
	//(*updateVelocityInn) = (*updateAccelerationInnENU + *previousAccelerationInnENU) * (interval/2) + (*velocityInn) ;
	
	CMatrix* dPositionENU = new CMatrix(3,1);
	(*dPositionENU) = (*updateVelocityInn) * ( interval);//*previousVelocityInn +
	
	CMatrix* geodeticPositionInn = new CMatrix(3,1);
	(*geodeticPositionInn)(0,0) = (*positionInn)(0,0) + conversion->Radian2Deg((*dPositionENU)(0,0) / ((N+height)*cos(phi))) ;
	(*geodeticPositionInn)(1,0) = (*positionInn)(1,0) + conversion->Radian2Deg((*dPositionENU)(1,0) / (M+height));
	(*geodeticPositionInn)(2,0) = (*positionInn)(2,0) + (*dPositionENU)(2,0);

	//(*geodeticPositionInn) = (*testPosition);
	delete dPositionENU;
	(*previousAccelerationInnENU)=(*updateAccelerationInnENU);
	delete updateAccelerationInnENU;
	
	(*positionInn) = (*geodeticPositionInn);
	(*velocityInn) = (*updateVelocityInn);
	delete win_n;
	delete geodeticPositionInn;
	delete updateVelocityInn;
	delete sensedAccelerationInb;
	delete sensedAngRatei2bInb;
}

//function: calculate Coriolis acceleration in e
//Parameters:	
//		veVIne			Velocity vector 
//		veCoriolis		Coriolis acceleration vector
void CIMUProcessing::CalcCoriolisIne(CMatrix& velocityIne,  CMatrix& coriolis)
{
	const double w_e= 7.2921150*0.00001;//(Earth Rotation Rate) radian per second
	coriolis(0,0) = -2 * w_e * velocityIne(1,0);
	coriolis(1,0) = 2 * w_e * velocityIne(0,0);
	coriolis(2,0) = 0.0;
}

void CIMUProcessing::UpdateQuaternion(CMatrix& veAnge2bInb, CMatrix& previousQuaternion , CMatrix& currentQuaternion)
{
	//Rotation angle antisymmetric quaternion Matrix 
	CConversion^ conversion = gcnew CConversion();
	double dAngNorm = conversion->NormVector(veAnge2bInb);
	//double dAngNorm = veAnge2bInb.GetNorm();
	CMatrix maS; maS.InitZeroMatrix(4,4);
	maS(0,0) = 0;				maS(0,1) = -veAnge2bInb(0,0);	maS(0,2) = -veAnge2bInb(1,0);	maS(0,3) = -veAnge2bInb(2,0);
	maS(1,0) = veAnge2bInb(0,0);	maS(1,1) = 0;				maS(1,2) =  veAnge2bInb(2,0);	maS(1,3) = -veAnge2bInb(1,0);
	maS(2,0) = veAnge2bInb(1,0);	maS(2,1) = -veAnge2bInb(2,0);	maS(2,2) = 0;				maS(2,3) = veAnge2bInb(0,0);
	maS(3,0) = veAnge2bInb(2,0);	maS(3,1) = veAnge2bInb(1,0);	maS(3,2) = -veAnge2bInb(0,0);	maS(3,3) = 0;
	
	//Intermediate variables
	f8 c = cos(dAngNorm/2.0);
	f8 s = sin((dAngNorm + EPSILON)/ 2.0) / (dAngNorm + EPSILON);

	//Calculation Quaternion vectors
	CMatrix maI;	maI.SetIdentity(4);
	currentQuaternion = ( maI * c + maS * s ) * previousQuaternion;
	
	//Quaternion Standardization 
	StandardizeQuaternion(currentQuaternion);
}

void CIMUProcessing::StandardizeQuaternion(CMatrix& quaternion)
{
	f8 dQua = sqrt(quaternion(0,0)*quaternion(0,0) + quaternion(1,0)*quaternion(1,0) + quaternion(2,0)*quaternion(2,0) + quaternion(3,0)*quaternion(3,0));//Quaternion Standardize
	for(int4 i=0; i<4; i++){quaternion(i,0) /= dQua;}
}

void CIMUProcessing::GetRb2LFromAttitude(CMatrix& attitude, CMatrix& maRb2L)
{
	f8 p,r,y; p = attitude(0,0) ; r = attitude(1,0) ; y = attitude(2,0) ;
	f8 dRb2L[9] = { cos(r)*cos(y) - sin(r)*sin(y)*sin(p),	-sin(y)*cos(p),	cos(y)*sin(r) + sin(y)*sin(p)*cos(r),
					cos(r)*sin(y) + sin(r)*cos(y)*sin(p),	cos(y)*cos(p),	sin(y)*sin(r) - cos(y)*sin(p)*cos(r),
					- cos(p)*sin(r),							sin(p),				cos(p)*cos(r)				};
	maRb2L.InitMatrix(dRb2L,3,3);
}

void CIMUProcessing::Initialization(CMatrix* initX, CMatrix* initV , CMatrix* initRb2n , CMatrix* initBiasAcc , CMatrix* initBiasGyro , CMatrix* initScaleFactorErrorAcc)
{
	(*positionInn) = (*initX);
	(*velocityInn) = (*initV);
	(*Rb2n) = (*initRb2n);

	(*biasAcc) = (*initBiasAcc);
	(*biasGyro) = (*initBiasGyro);
	(*scaleFactorErrorAcc) = (*initScaleFactorErrorAcc);
	//(*scaleFactorErrorGyro) = (*initScaleFactorErrorGyro);
}

void CIMUProcessing::Quaternion2R(CMatrix& quaternion, CMatrix& rotationMatrix)
{
	rotationMatrix(0,0) = quaternion(0,0)*quaternion(0,0)+quaternion(1,0)*quaternion(1,0)-quaternion(2,0)*quaternion(2,0)-quaternion(3,0)*quaternion(3,0);	
	rotationMatrix(1,0) = 2.0*(quaternion(1,0)*quaternion(2,0)-quaternion(0,0)*quaternion(3,0));
	rotationMatrix(2,0) = 2.0*(quaternion(1,0)*quaternion(3,0)+quaternion(0,0)*quaternion(2,0));

	rotationMatrix(0,1) = 2.0*(quaternion(1,0)*quaternion(2,0)+quaternion(0,0)*quaternion(3,0));
	rotationMatrix(1,1) = quaternion(0,0)*quaternion(0,0)-quaternion(1,0)*quaternion(1,0)+quaternion(2,0)*quaternion(2,0)-quaternion(3,0)*quaternion(3,0);	
	rotationMatrix(2,1) = 2.0*(quaternion(2,0)*quaternion(3,0)-quaternion(0,0)*quaternion(1,0));

	rotationMatrix(0,2) = 2.0*(quaternion(1,0)*quaternion(3,0)-quaternion(0,0)*quaternion(2,0));
	rotationMatrix(1,2) = 2.0*(quaternion(0,0)*quaternion(1,0)+quaternion(2,0)*quaternion(3,0));
	rotationMatrix(2,2) = quaternion(0,0)*quaternion(0,0)-quaternion(1,0)*quaternion(1,0)-quaternion(2,0)*quaternion(2,0)+quaternion(3,0)*quaternion(3,0);
}

void CIMUProcessing::GetRe2L(double dBInRadian, double dLInRadian, CMatrix& maRe2L)
{
	f8 dRe2L[9] = {	-sin(dLInRadian),					cos(dLInRadian),				0,
					-sin(dBInRadian)*cos(dLInRadian),		-sin(dBInRadian)*sin(dLInRadian),		cos(dBInRadian),
					cos(dBInRadian)*cos(dLInRadian),		cos(dBInRadian)*sin(dLInRadian),		sin(dBInRadian)};
	maRe2L.InitMatrix(dRe2L,3,3);
}

//Func: get a conversion matrix from local to ECEF frame
void CIMUProcessing::GetRL2e(double dBInRadian, double dLInRadian, CMatrix& maRL2e)
{
	f8 dRL2e[9] = { -sin(dLInRadian),						-sin(dBInRadian)*cos(dLInRadian),		cos(dBInRadian)*cos(dLInRadian),
					cos(dLInRadian),						-sin(dBInRadian)*sin(dLInRadian),		cos(dBInRadian)*sin(dLInRadian),
					0,										cos(dBInRadian),						sin(dBInRadian)};
	maRL2e.InitMatrix(dRL2e,3,3);
}

//function: using rotation matrix in calculating the quaternion
//Parameters:	
//		maR				rotation matrix
//		veQuaternion	quaternion vector
void CIMUProcessing::R2Quaternion(CMatrix& rotationMatrix, CMatrix& quaternion)
{
	//if (!( (rotationMatrix) (0,0)<=0 ||  (rotationMatrix) (0,0)>0))
	//	int ti11reriii=0;
	quaternion(0,0) = 0.5 * sqrt(1+rotationMatrix(0,0)+rotationMatrix(1,1)+rotationMatrix(2,2));// quaternion(0,0)
	quaternion(1,0) = (rotationMatrix(1,2)-rotationMatrix(2,1))/(4 * quaternion(0,0)) ;
	quaternion(2,0) = (rotationMatrix(2,0)-rotationMatrix(0,2))/(4 * quaternion(0,0));
	quaternion(3,0) = (rotationMatrix(0,1)-rotationMatrix(1,0))/(4 * quaternion(0,0));

}

void CIMUProcessing::IMUErrorRemoval(CMatrix* accelerometerMeasurement , CMatrix* gyroMeasurement)
{
	CMatrix* identityMatrix = new CMatrix(3,3);
	identityMatrix->SetIdentity(3);
	CMatrix* accScaleFactorErrorDiagonalMatrix = new CMatrix(3,3);
	(*accScaleFactorErrorDiagonalMatrix)(0,0) = (*scaleFactorErrorAcc)(0,0);
	(*accScaleFactorErrorDiagonalMatrix)(1,1) = (*scaleFactorErrorAcc)(1,0);
	(*accScaleFactorErrorDiagonalMatrix)(2,2) = (*scaleFactorErrorAcc)(2,0);

	//CMatrix* gyroScaleFactorErrorDiagonalMatrix = new CMatrix(3,3);
	//(*gyroScaleFactorErrorDiagonalMatrix)(0,0) = (*scaleFactorErrorGyro)(0,0);
	//(*gyroScaleFactorErrorDiagonalMatrix)(1,1) = (*scaleFactorErrorGyro)(1,0);
	//(*gyroScaleFactorErrorDiagonalMatrix)(2,2) = (*scaleFactorErrorGyro)(2,0);

	(*accelerometerMeasurement) = ((*identityMatrix) - (*accScaleFactorErrorDiagonalMatrix) ) * (*accelerometerMeasurement) - (*biasAcc);
	(*gyroMeasurement) =  (*gyroMeasurement) - (*biasGyro);

	delete identityMatrix;
	delete accScaleFactorErrorDiagonalMatrix;
	//delete gyroScaleFactorErrorDiagonalMatrix;
}