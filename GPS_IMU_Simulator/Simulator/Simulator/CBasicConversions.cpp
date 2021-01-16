#include "CBasicConversions.h"

CBasicConversions::CBasicConversions()
{
	
}

CBasicConversions::~CBasicConversions()
{
	
}

double CBasicConversions::Degree2Rad(double angleInDeg)
{
	double PI=3.14159265358979323846;
	double angleInRad=angleInDeg*PI/180.0;
	return angleInRad;

	
}

double CBasicConversions::Radian2Deg(double angleInRad)
{
	double PI=3.14159265358979323846;
	double angleInDeg=angleInRad*180.0/PI;
	return angleInDeg;
}

double CBasicConversions::PrimeMeridian(double latitude){
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(latitude);
	double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	//double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return M;
}

double CBasicConversions::PrimeVertical(double latitude){
	double f=1/298.257223563;
	double e2=2*f-f*f;
	double EarthA=6378137.0;
	double LatOriginiInRadian=Degree2Rad(latitude);
	//double M=EarthA*(1-e2)/(pow(1-e2*pow(sin(LatOriginiInRadian),2),1.5));
	double N=EarthA/(sqrt(1-e2*pow(sin(LatOriginiInRadian),2)));
	return N;
}

