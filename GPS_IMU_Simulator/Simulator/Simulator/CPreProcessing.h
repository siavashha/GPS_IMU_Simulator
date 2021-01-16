#include "CConfig.h"
#include "CMatrix.h"
//#include "SInitParam.h"
#include <math.h>
//#include "SConfigData.h"
//#include "SNetParam.h"
//#include "CNetParamCalc.h"

ref class CPreProcessing
{
private:

public:
	static int IMUgrade;
	static int GPStech;
	static double xOffset;
	static double yOffset;
	static double latOrigin;
	static double longOrigin;
	static double scale;
	static double azimuth;
	CPreProcessing();
	~CPreProcessing();
	void NetworkParamCalculator(String^);//CMatrix& extentLocal,CMatrix& extentGlobal,CMatrix& networkParam);
	void Geodetic2ENUExtent(CMatrix&  , CMatrix& , double , double);
	double Degree2Rad(double );
	double Radian2Deg(double);
	double PrimeMeridian(double );
	double PrimeVertical(double );
};