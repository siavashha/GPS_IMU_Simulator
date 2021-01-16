//#include "CMatrix.h"
//#include "SConfigData.h"
//#include "SConfigData.h"
#include "SNetParam.h"
//#include "SConfigData.h"
#include <math.h>

ref class CNetParamCalc
{
private:
	double m_scale;
	double m_azimuth;
	//CMatrix ;
public:
	CNetParamCalc();
	~CNetParamCalc();
	void NetworkParamCalc(SNetParam^ networkParamsData, SConfigData^ configData);// );// , SNetParam^ netParam
};