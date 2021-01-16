#include "CStateModel.h"
#include "CConversion.h"
//#include "CDisplayMatrixElements.h"

CStateModel::CStateModel()
{
	dynamicModelF = new CMatrix(21,21);
	systemNoiseModelG = new CMatrix(21,6);
	systemCovarianceMatrixQ = new CMatrix(6,6);
}

CStateModel::~CStateModel()
{
	delete dynamicModelF;
	delete systemNoiseModelG;
	delete systemCovarianceMatrixQ;
}

//function: According to the import value and internal preserve value get filtering results 
//Parameters:
void CStateModel::StateModelCalc(CMatrix* geodeticPositionInn, CMatrix* velocityInn , CMatrix* Rb2n , CMatrix* simulatedAccelerationInb , CMatrix* simulatedAngRate , int gradeIMU)
{
	const double w_e= 7.2921150*0.00001;//(Earth Rotation Rate) radian per second
	CConversion^ conversion = gcnew CConversion();
	double M = conversion->PrimeMeridian();
	double N = conversion->PrimeVertical();

	double velE = (*velocityInn)(0,0);
	double velN = (*velocityInn)(1,0);
	double velD = -(*velocityInn)(2,0);
		
	CMatrix* accelerationInn = new CMatrix(3,1);
	(*accelerationInn) = (*Rb2n) * (*simulatedAccelerationInb);
	double aN = (*accelerationInn)(0,0);
	double aE = (*accelerationInn)(1,0);
	double aD = (*accelerationInn)(2,0);

	double lambda = conversion->Degree2Rad((*geodeticPositionInn)(0,0));
	double phi = conversion->Degree2Rad((*geodeticPositionInn)(1,0));
	double height = (*geodeticPositionInn)(2,0);

	//Jekeli (2000), "Inertial Navigation Systems with Geodetic Applications" pp. 129 eq. 4.103
	double phiDot = velN / (M + height);
	double lambdaDot = velE / ((N + height) * cos(phi));
	double hDot = -velD;

	double radiusGaussian = sqrt( M * N );
	//gravity at this latitude and height
    double g0 = 9.7803267714 * ((1 + 0.001931851318513863 * sin(phi) * sin(phi)) / (sqrt(1 - 0.00669437999013 * sin(phi) * sin(phi))));
	//free air anamoly
	double g = g0 - 0.000003086 * height;

	//Jekeli (2000), "Inertial Navigation Systems with Geodetic Applications" pp. 155 
	double r =  radiusGaussian + height;

	//Jekeli (2000), "Inertial Navigation Systems with Geodetic Applications" pp. 162 eq. 5.82
	double F1array[7 * 7] = 
	{ 0 , -w_e * sin(phi) , 0 , 0 , cos(phi) , -w_e*sin(phi) , 0 ,
	w_e * sin(phi) , 0 , w_e * cos(phi) , -1 , 0 , 0 , 0 , 
	0 , -w_e * cos(phi) , 0 , 0 , -sin(phi) , -w_e * cos(phi) , 0 ,
	0 , g / r , 0 , 0 , -w_e * sin(2 * phi) , 0 , 0 ,
	-g / (r * cos(phi)) , 0 , 0 , 2 * w_e * tan(phi) , 0 , 0 , 0 ,
	0 , 0 , 0 , 1 , 0 , 0 , 0 , 
	0 , 0 , 0 , 0 , 1 , 0 , 0 };
	CMatrix* F1 = new CMatrix(F1array , 7 , 7);
	
	CMatrix* J = new CMatrix(2 , 3);
	(*J)(0,0) = 1;(*J)(0,1) = 0;(*J)(0,2) = 0;
	(*J)(1,0) = 0;(*J)(1,1) = 1;(*J)(1,2) = 0;

	CMatrix* invD = new CMatrix(3 , 3);
	(*invD)(0,0) = 1 / r; (*invD)(0,1) = 0; (*invD)(0,2) = 0;
	(*invD)(1,0) = 0; (*invD)(1,1) = 1 / (r * cos(phi)); (*invD)(1,2) = 0;
	(*invD)(2,0) = 0; (*invD)(2,1) = 0; (*invD)(2,2) = -1;

	CMatrix* diagAcc = new CMatrix(3 , 3);
	(*diagAcc)(0,0) = aN; (*diagAcc)(1,1) = aE; (*diagAcc)(2,2) = aD;

	CMatrix* F22 = new CMatrix(2 , 3);
	(*F22) = (*J) * (*invD) * (*Rb2n); 
	
	CMatrix* F23 = new CMatrix(2 , 3);
	(*F23) = (*J) * (*invD) * (*Rb2n) * (*diagAcc);
	//Jekeli (2000), "Inertial Navigation Systems with Geodetic Applications" pp. 313 eq. 10.34
	
	double F2array[7 * 9] = 
	{ -(*Rb2n)(0,0) , -(*Rb2n)(0,1) , -(*Rb2n)(0,2) , 0 , 0 , 0 , 0 , 0 , 0 ,
	  -(*Rb2n)(1,0) , -(*Rb2n)(1,1) , -(*Rb2n)(1,2) , 0 , 0 , 0 , 0 , 0 , 0 ,
	  -(*Rb2n)(2,0) , -(*Rb2n)(2,1) , -(*Rb2n)(2,2) , 0 , 0 , 0 , 0 , 0 , 0 ,

	0 , 0 , 0 , (*F22)(0,0) , (*F22)(0,1) , (*F22)(0,2) , (*F23)(0,0) , (*F23)(0,1) , (*F23)(0,2) , 
	0 , 0 , 0 , (*F22)(1,0) , (*F22)(1,1) , (*F22)(1,2) , (*F23)(1,0) , (*F23)(1,1) , (*F23)(1,2) ,

	0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

	CMatrix* F2 = new CMatrix(F2array , 7 , 9);
	CMatrix* F3 = new CMatrix(9,16);
	CMatrix* F = new CMatrix(7,7);	
	(*F)=(*F1);
	F->AppendMatrixInColumn(*F2);
	F->AppendMatrixInRow(*F3);

	(*dynamicModelF)=(*F);
	//data->MatrixShow(1,*F);

	double Garray[16 * 6] = 

	{ -(*Rb2n)(0,0) , -(*Rb2n)(0,1) , -(*Rb2n)(0,2) , 0 , 0 , 0 ,
	-(*Rb2n)(1,0) , -(*Rb2n)(1,1) , -(*Rb2n)(1,2) , 0 , 0 , 0 , 
	-(*Rb2n)(2,0) , -(*Rb2n)(2,1) , -(*Rb2n)(2,2) , 0 , 0 , 0 , 

	0 , 0 , 0 , (*F22)(0,0) , (*F22)(0,1) , (*F22)(0,2) ,  
	0 , 0 , 0 , (*F22)(1,0) , (*F22)(1,1) , (*F22)(1,2) ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,

	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 ,
	0 , 0 , 0 , 0 , 0 , 0 };

	CMatrix* G = new CMatrix(Garray , 16 , 6);
	(*systemNoiseModelG) = (*G);

	GetQ( gradeIMU);
	
	delete accelerationInn;
	delete diagAcc;
	delete invD;
	delete J;
	delete F1;
	delete F2;
	delete F22;
	delete F23;
	delete F3;
	delete F;	
	delete G;

}

void CStateModel::GetQ( int gradeIMU)
{
	if (gradeIMU==0)
	{
		(*systemCovarianceMatrixQ)(0,0) = 0.0002 * 0.0002;//0.2 milirad/second
		(*systemCovarianceMatrixQ)(1,1) = 0.0002 * 0.0002;
		(*systemCovarianceMatrixQ)(2,2) = 0.0002 * 0.0002;

		(*systemCovarianceMatrixQ)(3,3) = 0.0001 * 0.0001;//1 mili g
		(*systemCovarianceMatrixQ)(4,4) = 0.0001 * 0.0001;
		(*systemCovarianceMatrixQ)(5,5) = 0.0001 * 0.0001;
	}
	if (gradeIMU==1)
	{
		(*systemCovarianceMatrixQ)(0,0) = 1.5e-3 * 1.5e-3;
		(*systemCovarianceMatrixQ)(1,1) = 1.5e-3 * 1.5e-3;
		(*systemCovarianceMatrixQ)(2,2) = 1.5e-3 * 1.5e-3;

		(*systemCovarianceMatrixQ)(3,3) = 3e-7 * 3e-7;
		(*systemCovarianceMatrixQ)(4,4) = 3e-7 * 3e-7;
		(*systemCovarianceMatrixQ)(5,5) = 3e-7 * 3e-7;
	}
	if (gradeIMU==2)
	{
		(*systemCovarianceMatrixQ)(0,0) = 1.5e-3 * 1.5e-3;
		(*systemCovarianceMatrixQ)(1,1) = 1.5e-3 * 1.5e-3;
		(*systemCovarianceMatrixQ)(2,2) = 1.5e-3 * 1.5e-3;

		(*systemCovarianceMatrixQ)(3,3) = 3.5e-5 * 3.5e-5;
		(*systemCovarianceMatrixQ)(4,4) = 3.5e-5 * 3.5e-5;
		(*systemCovarianceMatrixQ)(5,5) = 3.5e-5 * 3.5e-5;
	}
	if (gradeIMU==3)
	{
		(*systemCovarianceMatrixQ)(0,0) = 0.01 * 0.01;
		(*systemCovarianceMatrixQ)(1,1) = 0.01 * 0.01;
		(*systemCovarianceMatrixQ)(2,2) = 0.01 * 0.01;

		(*systemCovarianceMatrixQ)(3,3) = 3e-4 * 3e-4;
		(*systemCovarianceMatrixQ)(4,4) = 3e-4 * 3e-4;
		(*systemCovarianceMatrixQ)(5,5) = 3e-4 * 3e-4;
	}
}
