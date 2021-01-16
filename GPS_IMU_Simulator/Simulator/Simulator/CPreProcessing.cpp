#include "CPreProcessing.h"
//#include "SConfigData.h"
//#define	double PI=3.14159265358979323846;//math.pi;
CPreProcessing::CPreProcessing()
{
	//IMUgrade = 1;
	//GPStech = 1;
	//xOffset = 0.0;
	//yOffset = 0.0;
	//latOrigin = 0.0;
	//longOrigin = 0.0;
	//scale = 1.0;
	//azimuth = 0.0;
}
CPreProcessing::~CPreProcessing()
{

}

void CPreProcessing::NetworkParamCalculator(String^ fileName)
{
	CConfig^ configFile=  gcnew CConfig( fileName );
	configFile->ReadConfig();
	
	CMatrix* localCartExtent = new CMatrix(*( configFile->localCartExtent));
	//configData->globalGeoExtent;
	CMatrix* globalGeoExtent= new CMatrix(*( configFile->globalGeoExtent));

	double latOrigin=( (*globalGeoExtent)(0,0)+
		(*globalGeoExtent)(1,0)+
		(*globalGeoExtent)(2,0)+
		(*globalGeoExtent)(3,0) )/4.0;//+configData->globalGeoExtent(1,0)+configData->globalGeoExtent(2,0)+configData->globalGeoExtent(3,0))/4.0;

	double longOrigin=( (*globalGeoExtent)(0,1)+
		(*globalGeoExtent)(1,1)+
		(*globalGeoExtent)(2,1)+
		(*globalGeoExtent)(3,1) )/4.0;
	
	double xOffset=( (*localCartExtent)(0,0)+
		(*localCartExtent)(1,0)+
		(*localCartExtent)(2,0)+
		(*localCartExtent)(3,0) )/4.0;

	double yOffset=( (*localCartExtent)(0,1)+
		(*localCartExtent)(1,1)+
		(*localCartExtent)(2,1)+
		(*localCartExtent)(3,1) )/4.0;
	

	CMatrix* localENUExtent = new CMatrix(4,2);
	Geodetic2ENUExtent( *globalGeoExtent , *localENUExtent , latOrigin , longOrigin);
	
	double sum1=0.0;
	double sum2=0.0;
	double sum3=0.0;
	for (int i=0;i<=3;i++){
		sum1=sum1+((*localENUExtent)(i,0))*((*localCartExtent)(i,0)-xOffset)+((*localENUExtent)(i,1))*((*localCartExtent)(i,1)-yOffset);
		sum2=sum2+((*localENUExtent)(i,0))*((*localCartExtent)(i,1)-yOffset)-((*localENUExtent)(i,1))*((*localCartExtent)(i,0)-xOffset);
		sum3=sum3+((*localCartExtent)(i,0)-xOffset)*((*localCartExtent)(i,0)-xOffset)+((*localCartExtent)(i,1)-yOffset)*((*localCartExtent)(i,1)-yOffset);
		}
	double a=sum1/sum3;
	double b=sum2/sum3;

	if(fabs(sum3) < 1.e-05)
		int ij = 1;
	
	double scale= sqrt(a*a+b*b);
	double azimuth=atan2(b,a); 

	this->IMUgrade=configFile->IMUgrade;
	this->GPStech=configFile->GPStech;
	this->xOffset=xOffset;
	this->yOffset=yOffset;
	this->latOrigin=latOrigin;
	this->longOrigin=longOrigin;
	this->scale=scale;
	this->azimuth=azimuth;
	//double PI=3.14159265358979323846;//math.pi;
	//if (azimuth>(PI/8))
	//{
	//	int i=0;
	//}
	delete localENUExtent;
	delete localCartExtent;
	delete globalGeoExtent;
}

void CPreProcessing::Geodetic2ENUExtent(CMatrix& geodeticCoord , CMatrix& enuCoord, double LatOrigin, double LongOrigin)
{
	// Global origin od the input data, // Earth's parameters,Curvature in meridinal and Prime vertical, Gauss mean curvature
	double PI=3.14159265358979323846;//math.pi;
	
	double M=PrimeMeridian(LatOrigin);
	double N=PrimeVertical(LatOrigin);

	double LatOriginiInRad=Degree2Rad(LatOrigin);

	for (int i=0;i<=geodeticCoord.GetRowNumber()-1;i++)
	{
		enuCoord(i,1)=Degree2Rad(geodeticCoord(i,0)-LatOrigin)*M;
		enuCoord(i,0)=Degree2Rad(geodeticCoord(i,1)-LongOrigin)*cos(LatOriginiInRad)*N;
		enuCoord(i,2)=geodeticCoord(i,2);
	}
}

double CPreProcessing::Degree2Rad(double angleInDeg)
{
	double PI=3.14159265358979323846;//
	double angleInRad=angleInDeg*PI/180.0;
	return angleInRad;
}

double CPreProcessing::Radian2Deg(double angleInRad)
{
	double PI=3.14159265358979323846;//
	double angleInDeg=angleInRad*180.0/PI;
	return angleInDeg;
}

double CPreProcessing::PrimeMeridian(double latitude){
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(latitude);
	double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	//double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return M;
}
double CPreProcessing::PrimeVertical(double latitude){
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(latitude);
	//double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return N;
}