#include "CAngularRateCalc.h"

CAngularRateCalc::CAngularRateCalc()
{
	timeT = new CMatrix(3,1);
	timeW = new CMatrix(3,1);

	carCurrentEulerAngle = new CMatrix(3,1);
	carPreviousEulerAngle = new CMatrix(3,1);
	carNextEulerAngles = new CMatrix(3,1);

	carAngularRate = new CMatrix(3,1);

	carCurrentAngularRate = new CMatrix(3,1);
	carPreviousAngularRate = new CMatrix(3,1);
	carNextAngularRate = new CMatrix(3,1);
}

CAngularRateCalc::~CAngularRateCalc()
{	
	delete timeT;
	delete timeW;

	delete carCurrentEulerAngle;
	delete carPreviousEulerAngle;
	delete carNextEulerAngles;

	delete carAngularRate;

	delete carCurrentAngularRate;
	delete carPreviousAngularRate;
	delete carNextAngularRate;
}

double CAngularRateCalc::CarAngularRateCalc(CMatrix* angle , double tAngle)
{
	//Console::WriteLine(tx);
	ReboundOrientation(angle,tAngle);
	if ((*timeT)(1,0)==0)
	{
		//the velocity cannot be calculated for one epoch and we need two positions to calculate velocity
		return 0;
	}
	if ((*timeT)(0,0)==0)
	{
		//second epoch: V1=X2-X1/t2-t1
		Differentatior(*carNextEulerAngles,*carCurrentEulerAngle,(*timeT)(2,0),(*timeT)(1,0),*carAngularRate);
	}
	else if ((*timeT)(2,0)==0)
	{
		//last epoch: Vn=Xn-X(n-1)/tn-t(n-1)
		Differentatior(*carCurrentEulerAngle,*carPreviousEulerAngle,(*timeT)(1,0),(*timeT)(0,0),*carAngularRate);
	}
	else 
	{
		// epoch: Vn=X(k+1)-X(k-1)/t(k+1)-t(k-1)
		Differentatior(*carCurrentEulerAngle,*carPreviousEulerAngle,(*timeT)(1,0),(*timeT)(0,0),*carAngularRate);
	}
	double tAngleRate=(*timeT)(1,0);
	ReboundAngularRate(carAngularRate, tAngleRate);
	return tAngleRate;
	//Console::WriteLine(tv);
}

void CAngularRateCalc::Differentatior(CMatrix& x2,CMatrix& x1,double t2,double t1, CMatrix& Xdot)
{
	double PI = 3.14159265358979323846;
	if (t2==t1)
		return;
	Xdot(0,0)=(x2(0,0)-x1(0,0))/(t2-t1);
	Xdot(1,0)=(x2(1,0)-x1(1,0))/(t2-t1);

	double dHeading = x2(2,0) - x1(2,0);

	Xdot(2,0)=(x2(2,0)-x1(2,0))/(t2-t1);

	if (dHeading>PI)
	{
		Xdot(2,0)=(dHeading-2*PI)/(t2-t1);
	}
	else if(dHeading<-PI)
	{
		Xdot(2,0)=(dHeading+2*PI)/(t2-t1);
	}
	else
	{
		Xdot(2,0)=dHeading/(t2-t1);
	}
}

void CAngularRateCalc::ReboundOrientation(CMatrix* angle, double tAngle)
{
	(*carPreviousEulerAngle) = (*carCurrentEulerAngle);
	(*carCurrentEulerAngle) = (*carNextEulerAngles);
	(*carNextEulerAngles) = (*angle);

	(*timeT)(0,0)=(*timeT)(1,0);
	(*timeT)(1,0)=(*timeT)(2,0);
	(*timeT)(2,0)=tAngle;
}

void CAngularRateCalc::ReboundAngularRate(CMatrix* angleRate, double tAngleRate)
{
	(*carPreviousAngularRate) = (*carCurrentAngularRate);
	(*carCurrentAngularRate) = (*carNextAngularRate);
	(*carNextAngularRate) = (*angleRate);

	(*timeW)(0,0)=(*timeW)(1,0);
	(*timeW)(1,0)=(*timeW)(2,0);
	(*timeW)(2,0)=tAngleRate;
}