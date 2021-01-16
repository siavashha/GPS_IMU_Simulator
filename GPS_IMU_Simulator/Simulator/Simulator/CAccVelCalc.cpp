#include "CAccVelCalc.h"

CAccVelCalc::CAccVelCalc()
{
	timeV = new CMatrix(3,1);

	carCurrentVelocity = new CMatrix(3,1);
	carPreviousVelocity = new CMatrix(3,1);
	carNextVelocity = new CMatrix(3,1);
	carAcceleration = new CMatrix(3,1);
}
CAccVelCalc::~CAccVelCalc()
{
	delete timeV;
	delete carCurrentVelocity;
	delete carPreviousVelocity;
	delete carNextVelocity;
	delete carAcceleration;
}

void CAccVelCalc::Integrator(CMatrix& x2,CMatrix& x1,double t2,double t1, CMatrix& Xdot)
{
	x2(0,0)=x1(0,0)+Xdot(0,0)*(t2-t1);
	x2(1,0)=x1(1,0)+Xdot(1,0)*(t2-t1);
	x2(2,0)=x1(2,0)+Xdot(2,0)*(t2-t1);
}

double CAccVelCalc::CarVelocityCalc(CMatrix* v , double tv )
{
	ReboundVelocity(v,tv);

	//if ((*timeV)(1,0)==0)
	//{
	//	//the velocity cannot be calculated for one epoch and we need two positions to calculate velocity
	//	return 0;
	//}
	if ((*timeV)(0,0)==0)
	{
		//second epoch: V1=X2-X1/t2-t1
		Integrator(*carNextVelocity,*carCurrentVelocity,(*timeV)(2,0),(*timeV)(1,0),*carAcceleration);
	}
	else if ((*timeV)(2,0)==0)
	{
		//last epoch: Vn=Xn-X(n-1)/tn-t(n-1)
		Integrator(*carCurrentVelocity,*carPreviousVelocity,(*timeV)(1,0),(*timeV)(0,0),*carAcceleration);
	}
	else 
	{
		// epoch: Vn=X(k+1)-X(k-1)/t(k+1)-t(k-1)
		Integrator(*carNextVelocity,*carPreviousVelocity,(*timeV)(2,0),(*timeV)(0,0),*carAcceleration);
	}
	
	double ta=(*timeV)(2,0);
	return ta;
}

void CAccVelCalc::ReboundVelocity(CMatrix* v, double tv)
{
	(*carPreviousVelocity) = (*carCurrentVelocity);
	(*carCurrentVelocity) = (*carNextVelocity);
	(*carNextVelocity) = (*v);

	(*timeV)(0,0)=(*timeV)(1,0);
	(*timeV)(1,0)=(*timeV)(2,0);
	(*timeV)(2,0)=tv;

}