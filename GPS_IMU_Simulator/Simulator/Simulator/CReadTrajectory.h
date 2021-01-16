using namespace System;
using namespace std;
#include <iostream>
#include <fstream>
using namespace System::IO;
#include "CMatrix.h"
//#include "CConversion.h"
#include "STrajData.h"
#include <math.h>

ref class CReadTrajectory 
{
private:
	String ^ m_FileName;
	StreamReader^ din;
public:
	double time;
	int carID;
	CMatrix* carCenterMass;
	CMatrix* carFrontBumper;
	CMatrix* carRearBumper;
	double bumper2bumperlength;
	double speed;
	double acceleration;
	int laneChangeIndicator1;
	int laneChangeIndicator2;
	wchar_t turnsign;
		
	STrajData^ trajectoryData;
	CReadTrajectory(String ^);
	~CReadTrajectory();
	bool ReadNextLine();
	void DataProvider();

};