#include "CConversion.h"

CConversion::CConversion()
{
	/*IMUgrade = this->IMUgrade;
	GPStech = this->GPStech;
	xOffset = this->xOffset;
	yOffset = this->yOffset;
	latOrigin = this->latOrigin;
	longOrigin = this->longOrigin;
	scale = this->scale;
	azimuth = this->azimuth;*/
}

CConversion::~CConversion()
{
}

double CConversion::Degree2Rad(double angleInDeg)
{
	double PI=3.14159265358979323846;
	double angleInRad=angleInDeg*PI/180.0;
	return angleInRad;
}

double CConversion::Radian2Deg(double angleInRad)
{
	double PI=3.14159265358979323846;
	double angleInDeg=angleInRad*180.0/PI;
	return angleInDeg;
}

double CConversion::PrimeMeridian(){

	double LatOrigin = this->latOrigin;	
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(LatOrigin);
	double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	//double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return M;
}

double CConversion::PrimeVertical(){

	double LatOrigin = this->latOrigin;	
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(LatOrigin);
	//double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return N;
}

void CConversion::ENU2Geodetic(CMatrix& geodeticCoord , CMatrix& enuCoord)
{
	double LatOrigin = this->latOrigin;	
	double LongOrigin = this->longOrigin;
	double M=PrimeMeridian();
	double N=PrimeVertical();

	double LatOriginiInRad=Degree2Rad(LatOrigin);

	geodeticCoord(0,0)=LongOrigin+Radian2Deg(enuCoord(0,0)/(cos(LatOriginiInRad)*N));
	geodeticCoord(1,0)=LatOrigin+Radian2Deg(enuCoord(1,0)/M);
	geodeticCoord(2,0)=enuCoord(2,0);
}

void CConversion::Geodetic2ENU(CMatrix& geodeticCoord , CMatrix& enuCoord)
{
	double LatOrigin = this->latOrigin;	
	double LongOrigin = this->longOrigin;
	double M=PrimeMeridian();
	double N=PrimeVertical();

	double LatOriginiInRad=Degree2Rad(LatOrigin);

	enuCoord(0,0)=Degree2Rad(geodeticCoord(0,0)-LongOrigin)*(cos(LatOriginiInRad)*N);
	enuCoord(1,0)=Degree2Rad(geodeticCoord(1,0)-LatOrigin)*M;
	enuCoord(2,0)=geodeticCoord(2,0);

}

void CConversion::local2ENU(CMatrix& localCoord , CMatrix& enuCoord)
{
	double Xm =	this->xOffset;
	double Ym = this->yOffset;
	//In order to test the program, this scale has been changed to one
	double scale =	1;//this ->scale;//////////////////1111
	double azimuth = this ->azimuth;
	
	CMatrix *localCoordRot = new CMatrix(3,1);
	double	dcm[3*3] = {scale*cos(azimuth),scale*sin(azimuth),0,-scale*sin(azimuth),scale*cos(azimuth),0,0,0,1};
	CMatrix *netOrientation = new CMatrix(dcm,3,3);
	(*localCoordRot)=(*netOrientation)*localCoord;
	/*double xoffset[3*1]={Xm,Ym,0};*/
	CMatrix *Xmoffset=new CMatrix(3,1);
	(*Xmoffset)(0,0)=Xm;
	(*Xmoffset)(1,0)=Ym;
	(*Xmoffset)(2,0)=0;
	(*Xmoffset)=(*netOrientation)*(*Xmoffset);
	enuCoord(0,0)=(*localCoordRot)(0,0)-(*Xmoffset)(0,0);
	enuCoord(1,0)=(*localCoordRot)(1,0)-(*Xmoffset)(1,0);
	enuCoord(2,0)=(*localCoordRot)(2,0);
	delete localCoordRot;
	delete netOrientation;
	delete Xmoffset;

	}

void CConversion::ENU2local( CMatrix& enuCoord,CMatrix& localCoord)
{

	// Global origin od the input data, // Earth's parameters,Curvature in meridinal and Prime vertical, Gauss mean curvature
	double inverseXm =	-1*(this->xOffset);
	double inverseYm = -1*(this->yOffset);
	double inverseScale = 1;//1.0/(this ->scale);
	double inverseAzimuth = -1*(this ->azimuth);
	CMatrix *enuCoord1= new CMatrix(3,1);
	double	dcm[3*3]={inverseScale*cos(inverseAzimuth),inverseScale*sin(inverseAzimuth),0,-inverseScale*sin(inverseAzimuth),inverseScale*cos(inverseAzimuth),0,0,0,1};
	CMatrix *netOrientation = new CMatrix(dcm,3,3);
	(*enuCoord1)=(*netOrientation)*enuCoord;
	localCoord(0,0)=(*enuCoord1)(0,0)-inverseXm;
	localCoord(1,0)=(*enuCoord1)(1,0)-inverseYm;
	localCoord(2,0)=(*enuCoord1)(2,0);
	
	delete enuCoord1;
	delete netOrientation;
}

double CConversion::NormVector( CMatrix& a)
{
	double sum = 0;
	for (int i=0;i<a.GetRowNumber();i++)
	{
		sum = sum + a(i,0) * a(i,0);
	}
	return sqrt(sum);
}

