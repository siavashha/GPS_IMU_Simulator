#include "CMatrix.h"
//#include "SConfigData.h"
using namespace System;
using namespace System::IO;
using namespace System::Collections;
using namespace std;

ref class CConfig
{
private:
	String^ m_fileName;
public:
	int IMUgrade;
	int GPStech;
	CMatrix* localCartExtent;
	CMatrix* globalGeoExtent;
	CConfig(String^ );
	~CConfig();
	void ReadConfig();
};


