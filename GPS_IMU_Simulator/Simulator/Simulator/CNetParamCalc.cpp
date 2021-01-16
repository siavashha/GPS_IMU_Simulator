#include "CNetParamCalc.h"

CNetParamCalc::CNetParamCalc()
{
	//m_fileName = gcnew String("");
}
CNetParamCalc::~CNetParamCalc()
{
	//delete m_fileName;
}

void CNetParamCalc::NetworkParamCalc(SNetParam^ networkParamsData,SConfigData^ configData)
	{
	//SConfigData^ configData){// , SNetParam^ netParam
	//CMatrix *extentENU=new CMatrix(4,2);
	//LatOrigin=(extentGlobal(0,0)+extentGlobal(1,0)+extentGlobal(2,0)+extentGlobal(3,0))/4.0;
	//LongOrigin=(extentGlobal(0,1)+extentGlobal(1,1)+extentGlobal(2,1)+extentGlobal(3,1))/4.0;
	//Geodetic2ENUExtent( extentGlobal , *extentENU);
	////displayMatrix(extentLocal);
	//double extentEm=((*extentENU)(0,0)+(*extentENU)(1,0)+(*extentENU)(2,0)+(*extentENU)(3,0))/4.0;
	//double extentNm=((*extentENU)(0,1)+(*extentENU)(1,1)+(*extentENU)(2,1)+(*extentENU)(3,1))/4.0;
	//double extentXm=(extentLocal(0,0)+extentLocal(1,0)+extentLocal(2,0)+extentLocal(3,0))/4.0;
	//double extentYm=(extentLocal(0,1)+extentLocal(1,1)+extentLocal(2,1)+extentLocal(3,1))/4.0;
	//
	//double sum1=0.0;
	//double sum2=0.0;
	//double sum3=0.0;
	//for (int i=0;i<=3;i++){
	//	sum1=sum1+((*extentENU)(i,0)-extentEm)*(extentLocal(i,0)-extentXm)+((*extentENU)(i,1)-extentNm)*(extentLocal(i,1)-extentYm);
	//	sum2=sum2+((*extentENU)(i,0)-extentEm)*(extentLocal(i,1)-extentYm)-((*extentENU)(i,1)-extentNm)*(extentLocal(i,0)-extentXm);
	//	sum3=sum3+(extentLocal(i,0)-extentXm)*(extentLocal(i,0)-extentXm)+(extentLocal(i,1)-extentYm)*(extentLocal(i,1)-extentYm);
	//	}
	//double a=sum1/sum3;
	//double b=sum2/sum3;

	//if(fabs(sum3) < 1.e-05)
	//	int ij = 1;
	//
	//double scale= sqrt(a*a+b*b);
	//double rotAngle=atan2(b,a); 
	//double scale2=((*extentENU)(2,0)-extentEm)/(extentLocal(2,0)-extentXm);
	//networkParam(0,0)=extentXm;
	//networkParam(1,0)=extentYm;
	//networkParam(2,0)=extentEm;
	//networkParam(3,0)=extentNm;
	//networkParam(4,0)=scale;
	//networkParam(5,0)=rotAngle;

	//delete extentENU;
	}