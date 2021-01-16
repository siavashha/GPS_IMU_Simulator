#include "CEstimatedErrorCorrection.h"
#include "CKalmanFilter.h"
#include "CConversion.h"

CEstimatedErrorCorrection::CEstimatedErrorCorrection()
{
	updatedRb2n = new CMatrix(3,3);
	updatedVelocityInn = new CMatrix(3,1);
	updatedGeodeticPositionInn = new CMatrix(3,1);
	updatedBiasAccelerometer = new CMatrix(3,1);
	updatedBiasGyro = new CMatrix(3,1);
	updatedScaleFactorErrorAccelerometer = new CMatrix(3,1);
	/*updatedScaleFactorErrorGyro = new CMatrix(3,1);*/
}

CEstimatedErrorCorrection::~CEstimatedErrorCorrection()
{
	delete updatedRb2n;
	delete updatedVelocityInn;
	delete updatedGeodeticPositionInn;
	delete updatedBiasAccelerometer;
	delete updatedBiasGyro;
	delete updatedScaleFactorErrorAccelerometer;
	/*delete updatedScaleFactorErrorGyro;*/
}

void CEstimatedErrorCorrection::ErrorCorrection(CMatrix* geodeticPositionInn , CMatrix* velocityInn,  CMatrix* Rb2n, CMatrix* biasAccelerometer, CMatrix* biasGyro, CMatrix* scaleFactorErrorAccelerometer , CMatrix* x)
{
	
	CConversion^ conversion = gcnew CConversion(); 
	CMatrix* stateVector = new CMatrix(21,1);
	(*stateVector)=(*x);
	double M = conversion->PrimeMeridian();
	double N = conversion->PrimeVertical();

	double velE = (*velocityInn)(0,0);
	double velN = (*velocityInn)(1,0);
	double velD = -(*velocityInn)(2,0);
		
	double lambda = conversion->Degree2Rad((*geodeticPositionInn)(0,0));
	double phi = conversion->Degree2Rad((*geodeticPositionInn)(1,0));
	double height = (*geodeticPositionInn)(2,0);

	double psi1;
	double psi2;
	double psi3;

	double dPhiDot;
	double dLambdaDot;
	//double dHDot;

	double dPhi;
	double dLambda;
	//double dH;

	double biasAccX;
	double biasAccY;
	double biasAccZ;

	double biasGyroX;
	double biasGyroY;
	double biasGyroZ;

	double scaleFactorAccX;
	double scaleFactorAccY;
	double scaleFactorAccZ;

	//double scaleFactorGyroX;
	//double scaleFactorGyroY;
	//double scaleFactorGyroZ;

	//Console::WriteLine((*stateVector)(6,0));
	psi1 = (*stateVector)(0,0);
	psi2 = (*stateVector)(1,0);
	psi3 = (*stateVector)(2,0);
	
	//Console::WriteLine("*************psi1  *********");
	//Console::WriteLine(psi1);

	dPhiDot = (*stateVector)(3,0);
	dLambdaDot = (*stateVector)(4,0);
	//dHDot = (*stateVector)(5,0);

	dPhi = (*stateVector)(5,0);
	dLambda = (*stateVector)(6,0);
	//dH = (*stateVector)(8,0);

	biasGyroX = (*stateVector)(7,0);
	biasGyroY = (*stateVector)(8,0);
	biasGyroZ = (*stateVector)(9,0);

	biasAccX = (*stateVector)(10,0);
	biasAccY = (*stateVector)(11,0);
	biasAccZ = (*stateVector)(12,0);

	scaleFactorAccX = (*stateVector)(13,0);
	scaleFactorAccY = (*stateVector)(14,0);
	scaleFactorAccZ = (*stateVector)(15,0);

	//scaleFactorGyroX = (*stateVector)(15,0);
	//scaleFactorGyroY = (*stateVector)(16,0);
	//scaleFactorGyroZ = (*stateVector)(20,0);

	CMatrix* eyeMatrix = new CMatrix(3,3);
	eyeMatrix->SetIdentity(3);

	double psiArray[3*3] = {0 , -psi3 , psi2 , psi3 , 0 , -psi1 , -psi2 , psi1 , 0}; 
	CMatrix* psi = new CMatrix(psiArray, 3,3);
	(*updatedRb2n) = (*Rb2n) * (*eyeMatrix - *psi);

	delete stateVector;
	delete  eyeMatrix;
	delete psi;
	//enu
	(*updatedVelocityInn)(0,0) = (*velocityInn)(0,0) + dLambdaDot *  (N + height) * cos(phi);
	(*updatedVelocityInn)(1,0) = (*velocityInn)(1,0) + dPhiDot *  (M + height);
	(*updatedVelocityInn)(2,0) = (*velocityInn)(2,0);

	//CDisplayMatrixElements^ display = gcnew CDisplayMatrixElements();
	//Console::WriteLine((*geodeticPositionInn)(0,0));
	
	//Console::WriteLine("dH");
	//Console::WriteLine(dH);	//BLH
	(*updatedGeodeticPositionInn)(0,0) = (*geodeticPositionInn)(0,0) + conversion->Radian2Deg(dLambda);
	(*updatedGeodeticPositionInn)(1,0) = (*geodeticPositionInn)(1,0) + conversion->Radian2Deg(dPhi);
	(*updatedGeodeticPositionInn)(2,0) = (*geodeticPositionInn)(2,0) ;
	//Console::WriteLine((*geodeticPositionInn)(0,0));


	//Bias Accelerometer
	(*updatedBiasAccelerometer)(0,0) = biasAccX;
	(*updatedBiasAccelerometer)(1,0) = biasAccY;
	(*updatedBiasAccelerometer)(2,0) = biasAccZ;

	//Bias Gyro
	(*updatedBiasGyro)(0,0) = biasGyroX;
	(*updatedBiasGyro)(1,0) = biasGyroY;
	(*updatedBiasGyro)(2,0) = biasGyroZ;

	//SC Accelerometer
	(*updatedScaleFactorErrorAccelerometer)(0,0) = scaleFactorAccX;
	(*updatedScaleFactorErrorAccelerometer)(1,0) = scaleFactorAccY;
	(*updatedScaleFactorErrorAccelerometer)(2,0) = scaleFactorAccZ;

	//SC Gyro
	//(*updatedScaleFactorErrorGyro)(0,0) = scaleFactorGyroX;
	//(*updatedScaleFactorErrorGyro)(1,0) = scaleFactorGyroY;
	//(*updatedScaleFactorErrorGyro)(2,0) = scaleFactorGyroZ;
}