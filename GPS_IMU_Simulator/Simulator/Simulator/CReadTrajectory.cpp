#include "CReadTrajectory.h"

CReadTrajectory::CReadTrajectory(String ^ fileName)
{
	m_FileName = fileName;
	trajectoryData=  gcnew STrajData();
	din = gcnew StreamReader(m_FileName);

	carCenterMass = new CMatrix(3,1);
	carFrontBumper = new CMatrix(3,1);
	carRearBumper = new CMatrix(3,1);

	time = 0;
	carID = 0;
	bumper2bumperlength = 0;
	speed = 0;
	acceleration = 0;
	laneChangeIndicator1 = 0;
	laneChangeIndicator2 = 0;
	turnsign = NULL;
}

CReadTrajectory::~CReadTrajectory()
{
	delete m_FileName;
	delete trajectoryData;
	delete din;

	delete carCenterMass;
	delete carFrontBumper;
	delete carRearBumper;
}

bool CReadTrajectory::ReadNextLine()
{
if (!(File::Exists(m_FileName)))
{return false;}
String^ strCurrentLine;
array<String^>^ tempStr;
//read the file line by line
if ((strCurrentLine = din->ReadLine()) != nullptr) 
	{
	tempStr=strCurrentLine->Split( ',' );
	//File information: time,Vehicle ID,rear Bumper XYZ, front Bumper XYZ
	trajectoryData->time=Convert::ToDouble(tempStr[0]);
	trajectoryData->carID=Convert::ToInt32(tempStr[1]);
	trajectoryData->centerMassX=Convert::ToDouble(tempStr[2]);
	trajectoryData->centerMassY=Convert::ToDouble(tempStr[3]);
	trajectoryData->centerMassZ=Convert::ToDouble(tempStr[4]);
	trajectoryData->frontBumperX=Convert::ToDouble(tempStr[5]);
	trajectoryData->frontBumperY=Convert::ToDouble(tempStr[6]);
	trajectoryData->frontBumperZ=Convert::ToDouble(tempStr[7]);
	trajectoryData->rearBumperX=Convert::ToDouble(tempStr[8]);
	trajectoryData->rearBumperY=Convert::ToDouble(tempStr[9]);
	trajectoryData->rearBumperZ=Convert::ToDouble(tempStr[10]);
	trajectoryData->bumper2bumperlength=Convert::ToDouble(tempStr[11]);
	trajectoryData->speed=Convert::ToDouble(tempStr[12]);
	trajectoryData->acceleration=Convert::ToDouble(tempStr[13]);
	trajectoryData->laneChangeIndicator1=Convert::ToInt32(tempStr[14]);
	trajectoryData->laneChangeIndicator2=Convert::ToInt32(tempStr[15]);
	trajectoryData->turnsign=Convert::ToChar(tempStr[16]);
	}
	else {
		trajectoryData->time=0.0;
		return false;}
	return true;
}

void CReadTrajectory::DataProvider()
{
	time = trajectoryData->time;
	carID = trajectoryData->carID;

	(*carCenterMass)(0,0)=trajectoryData->centerMassX;
	(*carCenterMass)(1,0)=trajectoryData->centerMassY;
	(*carCenterMass)(2,0)=trajectoryData->centerMassZ;
	
	(*carFrontBumper)(0,0)=trajectoryData->frontBumperX;
	(*carFrontBumper)(1,0)=trajectoryData->frontBumperY;
	(*carFrontBumper)(2,0)=trajectoryData->frontBumperZ;
	
	(*carRearBumper)(0,0)=trajectoryData->rearBumperX;
	(*carRearBumper)(1,0)=trajectoryData->rearBumperY;
	(*carRearBumper)(2,0)=trajectoryData->rearBumperZ;
	
	bumper2bumperlength = trajectoryData->bumper2bumperlength;
	speed = trajectoryData->speed;
	acceleration = trajectoryData->acceleration;
	laneChangeIndicator1 = trajectoryData->laneChangeIndicator1;
	laneChangeIndicator2 = trajectoryData->laneChangeIndicator2;
	turnsign = trajectoryData->turnsign;
}