#include "CConfig.h"

CConfig::CConfig(String^ fileName )
{
	m_fileName = fileName;
	int IMUgrade = 1;
	int GPStech = 1;
	localCartExtent = new CMatrix(4,2);
	globalGeoExtent = new CMatrix(4,2);
}
CConfig::~CConfig()
{
	delete m_fileName;
	delete localCartExtent;
	delete globalGeoExtent;
	//delete m_configData;
}
void CConfig::ReadConfig()
{
	StreamReader^ configFile = gcnew StreamReader(m_fileName);
	String^ str;
	while((str = configFile->ReadLine()) != nullptr) {
		if (str=="IMU grade"){
			str = configFile->ReadLine();
			this->IMUgrade=Convert::ToInt32(str);}
		else if (str=="GPS technique"){
			str = configFile->ReadLine();
			this->GPStech=Convert::ToInt32(str);}
		/*else if (str=="Cutoff Angle"){
			str = configFile->ReadLine();
			CutOffAngle=Convert::ToDouble(str);}
		else if (str=="IMU sampling rate with respect to GPS"){
			str = configFile->ReadLine();
			SamplingRate=Convert::ToDouble(str);}
		else if (str=="Network Rotation Angle"){
			str = configFile->ReadLine();
			NetworkRotAng=Convert::ToDouble(str);}*/

		double x1,x2,x3,x4,y1,y2,y3,y4;
		double Lat1,Long1,Lat2,Long2,Lat3,Long3,Lat4,Long4;
		if (str=="X     ,Y     (clockwise from WN point)"){
			array<Char>^sepDir = gcnew array<Char>{','};
			array<String^>^ tempstr;
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			x1=Convert::ToDouble(tempstr[0]);
			y1=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			x2=Convert::ToDouble(tempstr[0]);
			y2=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			x3=Convert::ToDouble(tempstr[0]);
			y3=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			x4=Convert::ToDouble(tempstr[0]);
			y4=Convert::ToDouble(tempstr[1]);}
		if (str=="Latitude (N), Longitude (E)")
		{
			array<Char>^sepDir = gcnew array<Char>{','};
			array<String^>^ tempstr;
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			Lat1=Convert::ToDouble(tempstr[0]);
			Long1=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			Lat2=Convert::ToDouble(tempstr[0]);
			Long2=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			Lat3=Convert::ToDouble(tempstr[0]);
			Long3=Convert::ToDouble(tempstr[1]);
			str = configFile->ReadLine();
			tempstr=str->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
			Lat4=Convert::ToDouble(tempstr[0]);
			Long4=Convert::ToDouble(tempstr[1]);
		}
	double xlocal[4*2]={x1, y1,
		x2, y2,
		x3, y3,
		x4, y4};
	double Geo[4*2]={Lat1, Long1,
		Lat2, Long2,
		Lat3, Long3,
		Lat4, Long4};
	
		this->localCartExtent=new CMatrix(xlocal,4,2);
		this->globalGeoExtent=new CMatrix(Geo,4,2);
	}
}



	