#include "CKalmanFilter.h"
#include "CDisplayMatrixElements.h"

CKalmanFilter::CKalmanFilter()
{
	timeModelUpdate = gcnew CStateModel();
	observationModelUpdate = gcnew CObservationModel();
	x = new CMatrix(16,1);
	P = new CMatrix(16,16);
	P->SetIdentity(16);
	dout = gcnew StreamWriter("test2.txt");
}

CKalmanFilter::~CKalmanFilter()
{	
	dout->Close();
	delete dout;
	delete timeModelUpdate;
	delete observationModelUpdate;
	delete x;
	delete P;
}

void CKalmanFilter::TimeUpdate(CMatrix* geodeticPositionInn , CMatrix* velocityInn , CMatrix* Rb2n , CMatrix* simulatedAccelerationInb  , CMatrix* simulatedAngRate , int gradeIMU , double dt)
{
	timeModelUpdate->StateModelCalc(geodeticPositionInn , velocityInn , Rb2n ,  simulatedAccelerationInb  , simulatedAngRate , gradeIMU);
	CMatrix* F = new CMatrix(16,16);
	(*F) = (*timeModelUpdate->dynamicModelF);
	CMatrix* transitionMatrix = new CMatrix(16,16);
	CMatrix* identityMatrix = new CMatrix(16,16);
	identityMatrix->SetIdentity(16);
	(*transitionMatrix) = (*identityMatrix) + (*F) * dt + (*F) * (*F) * (0.5 * dt * dt);
	CMatrix* G = new CMatrix(16,6);
	(*G) = (*timeModelUpdate->systemNoiseModelG);
	CMatrix* Q = new CMatrix(6,6);
	(*Q) = (*timeModelUpdate->systemCovarianceMatrixQ) * dt;
	(*x) = (*transitionMatrix) * (*x);
	(*P) =  (*transitionMatrix) * (*P) * (transitionMatrix->GetTranspose()) + (*G) * (*Q) * (G->GetTranspose());//
	(*P) = ((*P) + P->GetTranspose()) / 2;

	delete identityMatrix;
	delete F;
	delete transitionMatrix;
	delete G;
	delete Q;
}

void CKalmanFilter::ObservationUpdate( int gradeGPS , CMatrix* simulatedGPSPosition , CMatrix* simulatedIMUPosition)
{

	double dLambda = this->Degree2Rad((*simulatedGPSPosition)(0,0)-(*simulatedIMUPosition)(0,0));
	double dPhi = this->Degree2Rad((*simulatedGPSPosition)(1,0)-(*simulatedIMUPosition)(1,0));
	double dHeight = (*simulatedGPSPosition)(2,0)-(*simulatedIMUPosition)(2,0);
	double obsArray[2*1] = {dPhi , dLambda };
	CMatrix* dy = new CMatrix(obsArray,2,1);
	observationModelUpdate->ObservationModelCalc( gradeGPS );
	CMatrix* H = new CMatrix(2,16);
	(*H) = (*observationModelUpdate->designMatrixH);
	CMatrix* R= new CMatrix(2,2);
	(*R) = (*observationModelUpdate->observationNoiseModelR);
	CMatrix* gainK = new CMatrix(16,2);
	CMatrix* HPHplusR = new CMatrix(2,2);
	CMatrix* invHPHplusR = new CMatrix(2,2);
	//Console::WriteLine("************* Obs *********");
	//Console::WriteLine(dPhi);
	//Console::WriteLine(dLambda);
	(*HPHplusR) = (*H) * (*P) * (H->GetTranspose()) + (*R) ; //

	(*invHPHplusR) = (HPHplusR->GetInverse());
	(*gainK) = (*P) * (H->GetTranspose()) * (*invHPHplusR);

	(*x) = (*x) + (*gainK) * ((*dy) - (*H) * (*x));//
	
	CMatrix* IminusKH = new CMatrix(2,2);
	CMatrix* identityMatrix = new CMatrix(16,16);
	identityMatrix->SetIdentity(16);
	(*IminusKH) = (*identityMatrix) - (*gainK) * (*H);

	(*P) = (*IminusKH) * (*P) * (IminusKH->GetTranspose()) +  (*gainK) * (*R) * (gainK->GetTranspose());// 
	(*P) = ((*P) + (P->GetTranspose())) / 2;
	delete dy;
	delete H;
	delete R;
	delete gainK;
	delete IminusKH;
	delete HPHplusR;
	delete invHPHplusR;
	delete identityMatrix;
}

double CKalmanFilter::Degree2Rad(double angleInDeg)
{
	double PI=3.14159265358979323846;
	double angleInRad=angleInDeg*PI/180.0;
	return angleInRad;
}