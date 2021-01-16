#include "CExportTrajectory.h"
#include "CConversion.h"

CExportTrajectory::CExportTrajectory(String^ inputDirectory){
	m_inputDirectory = inputDirectory;
}

CExportTrajectory::~CExportTrajectory(){
	delete  m_inputDirectory;
}

void CExportTrajectory::WriteTrajectoryContent(double time, int carID, CMatrix* geodeticPosition, CMatrix* Rb2n, double frontBumperLength , double rearBumperLength, CMatrix* inventory)
{
	CMatrix* enuPosition = new CMatrix(3,1);
	CMatrix* xyzPosition = new CMatrix(3,1);
	CConversion^ conversion = gcnew CConversion(); 
	conversion->Geodetic2ENU(*geodeticPosition,*enuPosition);
	conversion->ENU2local(*enuPosition , *xyzPosition);

	double bumper2bumperlength = (*inventory)(0,5);
	double speed = (*inventory)(0,0);
	double acceleration = (*inventory)(0,1);
	int  laneChangeIndicator1 = (int)(*inventory)(0,2);
	int laneChangeIndicator2 = (int)(*inventory)(0,3);
	wchar_t trunsign = (wchar_t)(*inventory)(0,4);

	double frontBumperInbArray [3] = { frontBumperLength , 0 , 0 };
	CMatrix* frontBumperInb = new CMatrix(frontBumperInbArray,3,1);
	double rearBumperInbArray [3] = { -rearBumperLength , 0 , 0 };
	CMatrix* rearBumperInb = new CMatrix(rearBumperInbArray,3,1);

	CMatrix* frontBumperInnNED = new CMatrix(3,1);
	CMatrix* rearBumperInnNED = new CMatrix(3,1);

	(*frontBumperInnNED) = (*Rb2n) * (*frontBumperInb);
	
	double xFrontBumper = (*frontBumperInnNED)(1,0) + (*xyzPosition)(0,0);
	double yFrontBumper = (*frontBumperInnNED)(0,0) + (*xyzPosition)(1,0);
	double zFrontBumper = -(*frontBumperInnNED)(2,0) + (*xyzPosition)(2,0);
	
	(*rearBumperInnNED) = (*Rb2n) * (*rearBumperInb);
	double xRearBumper = (*rearBumperInnNED)(1,0) + (*xyzPosition)(0,0);
	double yRearBumper = (*rearBumperInnNED)(0,0) + (*xyzPosition)(1,0);
	double zRearBumper = -(*rearBumperInnNED)(2,0) + (*xyzPosition)(2,0);

 	dout->WriteLine(time+ "," + carID + "," + (*xyzPosition)(0,0) + "," + (*xyzPosition)(1,0) + "," + (*xyzPosition)(2,0) + "," + xFrontBumper + "," + yFrontBumper + "," + zFrontBumper + "," + xRearBumper + "," + yRearBumper + "," + zRearBumper + "," + bumper2bumperlength + "," + speed + "," + acceleration + "," + laneChangeIndicator1 + "," + laneChangeIndicator2 + "," + trunsign);//
	delete frontBumperInb;
	delete rearBumperInb;
	delete frontBumperInnNED;
	delete rearBumperInnNED;
	delete enuPosition;
	delete xyzPosition;
	delete conversion;
	dout->Flush();
}

String^ CExportTrajectory::TrajectoryFileNameProvider(String^ m_inputDirectory)
{
	array<Char>^sepDir = gcnew array<Char>{'/'};
	array<String^>^ tempStr;
	tempStr=m_inputDirectory->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
	int fileNameOrder = tempStr->Length;
	String^ fileName = tempStr[fileNameOrder-1];
	String^ fileNameIMU = "..\\Output\\Simulated Trajectory\\" + fileName->Remove(fileName->Length-4) + "_Output.txt";
	return fileNameIMU;
}

void CExportTrajectory::TrajectoryFileOpen()
{
	String^ fileName=TrajectoryFileNameProvider(m_inputDirectory);
	dout = gcnew StreamWriter(fileName);
}

void CExportTrajectory::TrajectoryFileClose()
{
	dout->Close();
	delete dout;
}
		
void CExportTrajectory::WriteFirstTrajectoryPoint(double time, int carID, CMatrix* carPosXyz, CMatrix* carFrontPos, CMatrix* carRearPos , double bumper2bumperlength , double speed, double acceleration , int laneChangeIndicator1 , int laneChangeIndicator2, wchar_t trunsign)
{
	dout->WriteLine(time+ "," + carID + "," + (*carPosXyz)(0,0) + "," + (*carPosXyz)(1,0) + "," + (*carPosXyz)(2,0) + "," + (*carFrontPos)(0,0) + "," + (*carFrontPos)(1,0) + "," + (*carFrontPos)(2,0) + "," + (*carRearPos)(0,0) + "," +  (*carRearPos)(1,0) + "," +  (*carRearPos)(2,0) + "," + bumper2bumperlength + "," + speed + "," + acceleration + "," + laneChangeIndicator1 + "," + laneChangeIndicator2 + "," + trunsign);//
	dout->Flush();
}





