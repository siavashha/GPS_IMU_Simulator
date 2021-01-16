#include "CExportIMUFile.h"

CExportIMUFile::CExportIMUFile(String^ inputDirectory){
	m_inputDirectory = inputDirectory;
}

CExportIMUFile::~CExportIMUFile(){
	delete  m_inputDirectory;
}

void CExportIMUFile::WriteIMUFileContent(double time, CMatrix* acceleration, CMatrix* angularRate)
{
	dIMUout->WriteLine( time + " " + (*acceleration)(0,0) + " " + (*acceleration)(1,0) + " " + (*acceleration)(2,0) + " " + (*angularRate)(0,0) + " " + (*angularRate)(1,0) + " " + (*angularRate)(2,0) );
	dIMUout->Flush();
}

String^ CExportIMUFile::IMUFileNameProvider(String^ m_inputDirectory)
{
	array<Char>^sepDir = gcnew array<Char>{'/'};
	array<String^>^ tempStr;
	tempStr=m_inputDirectory->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
	int fileNameOrder = tempStr->Length;
	String^ fileName = tempStr[fileNameOrder-1];
	String^ fileNameIMU = "..\\Output\\Simulated IMU\\" + fileName->Remove(fileName->Length-4) + "_IMU.csv";
	return fileNameIMU;
}

void CExportIMUFile::IMUFileOpen()
{
	String^ fileName=IMUFileNameProvider(m_inputDirectory);
	dIMUout = gcnew StreamWriter(fileName);
}

void CExportIMUFile::IMUFileClose()
{
	dIMUout->Close();
	delete dIMUout;
}