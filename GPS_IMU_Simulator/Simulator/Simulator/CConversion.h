//#include "CConfig.h"
#include "CMatrix.h"
//#include "SInitParam.h"
#include "CPreProcessing.h"
#include "CDisplayMatrixElements.h"
#include <math.h>


ref class CConversion : public CPreProcessing
{
private:
	String^ m_fileName;
	//SInitParam^ networkParamsData;  
	//int IMUgrade;
	//int GPStech;
	//double xOffset;
	//double yOffset;
	//double latOrigin;
	//double longOrigin;
	//double scale;
	//double azimuth;

public:
	
	CConversion();
	~CConversion();
	void Geodetic2ENU(CMatrix&  , CMatrix& );
	void ENU2Geodetic(CMatrix& , CMatrix& );
	double Degree2Rad(double );
	double Radian2Deg(double);
	double PrimeMeridian();
	double PrimeVertical();
	void local2ENU(CMatrix&  , CMatrix& );
	void ENU2local( CMatrix& ,CMatrix&);
	double NormVector( CMatrix&);

};