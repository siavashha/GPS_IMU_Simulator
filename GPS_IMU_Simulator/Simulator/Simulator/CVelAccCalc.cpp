#include "CVelAccCalc.h"

CVelAccCalc::CVelAccCalc()
{
	timeV = new CMatrix(3,1);

	carCurrentVelocity = new CMatrix(3,1);
	carPreviousVelocity = new CMatrix(3,1);
	carNextVelocity = new CMatrix(3,1);

	carAcceleration = new CMatrix(3,1);
}
CVelAccCalc::~CVelAccCalc()
{
	delete timeV;

	delete carCurrentVelocity;
	delete carPreviousVelocity;
	delete carNextVelocity;

	delete carAcceleration;
}

void CVelAccCalc::Differentatior(CMatrix& x2,CMatrix& x1,double t2,double t1, CMatrix& Xdot)
{
	Xdot(0,0)=(x2(0,0)-x1(0,0))/(t2-t1);
	Xdot(1,0)=(x2(1,0)-x1(1,0))/(t2-t1);
	Xdot(2,0)=(x2(2,0)-x1(2,0))/(t2-t1);


	double a1 = x2(0,0);
	double a2 = x1(0,0);

	double a= Xdot(0,0);
	double b = t2-t1;

}

double CVelAccCalc::CarAccelerationCalc(CMatrix* v , double tv )
{
	ReboundVelocity(v,tv);

	if ((*timeV)(1,0)==0)
	{
		//the velocity cannot be calculated for one epoch and we need two positions to calculate velocity
		return 0;
	}
	if ((*timeV)(0,0)==0)
	{
		//second epoch: V1=X2-X1/t2-t1
		Differentatior(*carNextVelocity,*carCurrentVelocity,(*timeV)(2,0),(*timeV)(1,0),*carAcceleration);
	}
	else if ((*timeV)(2,0)==0)
	{
		//last epoch: Vn=Xn-X(n-1)/tn-t(n-1)
		Differentatior(*carCurrentVelocity,*carPreviousVelocity,(*timeV)(1,0),(*timeV)(0,0),*carAcceleration);
	}
	else 
	{
		// epoch: Vn=X(k+1)-X(k-1)/t(k+1)-t(k-1)
		Differentatior(*carCurrentVelocity,*carPreviousVelocity,(*timeV)(1,0),(*timeV)(0,0),*carAcceleration);
		//Console::WriteLine("//Acceleration//");
		//Console::WriteLine((*carAcceleration)(0,0));
		//Console::WriteLine((*carAcceleration)(1,0));
		//Console::WriteLine((*carAcceleration)(2,0));
	}
	
	double ta=(*timeV)(1,0);
	return ta;
}

void CVelAccCalc::ReboundVelocity(CMatrix* v, double tv)
{
	(*carPreviousVelocity) = (*carCurrentVelocity);
	(*carCurrentVelocity) = (*carNextVelocity);
	(*carNextVelocity) = (*v);

	(*timeV)(0,0)=(*timeV)(1,0);
	(*timeV)(1,0)=(*timeV)(2,0);
	(*timeV)(2,0)=tv;

}