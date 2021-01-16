#include "CVelPosCalc.h"

CVelPosCalc::CVelPosCalc()
{
	timeX = new CMatrix(3,1);
	carCurrentPosition = new CMatrix(3,1);
	carPreviousPosition = new CMatrix(3,1);
	carNextPosition = new CMatrix(3,1);
	carVelocity = new CMatrix(3,1);
}

CVelPosCalc::~CVelPosCalc()
{
	delete timeX;
	delete carCurrentPosition;
	delete carPreviousPosition;
	delete carNextPosition;
	delete carVelocity;
}

void CVelPosCalc::Integrator(CMatrix& x2,CMatrix& x1,double t2,double t1, CMatrix& Xdot)
{
	x2(0,0)=x1(0,0)+Xdot(0,0)*(t2-t1);
	x2(1,0)=x1(1,0)+Xdot(1,0)*(t2-t1);
	x2(2,0)=x1(2,0)+Xdot(2,0)*(t2-t1);
}

double CVelPosCalc::CarPositionCalc(CMatrix* x , double tx)
{
	ReboundPosition(x,tx);
	//if ((*timeX)(1,0)==0)
	//{
	//	//the velocity cannot be calculated for one epoch and we need two positions to calculate velocity
	//	return 0;
	//}
	if ((*timeX)(0,0)==0)
	{
		//second epoch: V1=X2-X1/t2-t1
		Integrator(*carNextPosition,*carCurrentPosition,(*timeX)(2,0),(*timeX)(1,0),*carVelocity);
	}

	else if ((*timeX)(2,0)==0)
	{
		//last epoch: Vn=Xn-X(n-1)/tn-t(n-1)
		Integrator(*carCurrentPosition,*carPreviousPosition,(*timeX)(1,0),(*timeX)(0,0),*carVelocity);
	}
	else 
	{
		// epoch: Vn=X(k+1)-X(k-1)/t(k+1)-t(k-1)
		Integrator(*carNextPosition,*carPreviousPosition,(*timeX)(2,0),(*timeX)(0,0),*carVelocity);
	}
	double tv=(*timeX)(1,0);
	return tv;
	//Console::WriteLine(tv);
}

void CVelPosCalc::ReboundPosition(CMatrix* x, double t)
{
	(*carPreviousPosition) = (*carCurrentPosition);
	(*carCurrentPosition) = (*carNextPosition);
	(*carNextPosition) = (*x);

	(*timeX)(0,0)=(*timeX)(1,0);
	(*timeX)(1,0)=(*timeX)(2,0);
	(*timeX)(2,0)=t;
	
}
