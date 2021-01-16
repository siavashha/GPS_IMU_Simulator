#include "CMatrix.h"
ref struct SConfigData
{
	int IMUgrade;
	int GPStech;
	CMatrix* localCartExtent;
	CMatrix* globalGeoExtent;
public:
	SConfigData()
	{
		IMUgrade=1;
		GPStech=1;
		localCartExtent=new CMatrix(4,2);
		globalGeoExtent=new CMatrix(4,2);
	}
	~SConfigData()
	{
		delete localCartExtent;
		delete globalGeoExtent;
	}
};