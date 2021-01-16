#include "CExportGPSFile.h"

CExportGPSFile::CExportGPSFile(String^ inputDirectory){
	m_inputDirectory = inputDirectory;
}
CExportGPSFile::~CExportGPSFile(){
	delete  m_inputDirectory;
}

void CExportGPSFile::WriteGPSFileContent(double time, CMatrix* position, CMatrix* velocity)
{
	//Latitude, Longitude, height
	dGPSout->WriteLine("1444 " + time + " " + (*position)(1,0) + " " + (*position)(0,0) + " " + (*position)(2,0) + " " + "0.1 0.1 0.3 " + (*velocity)(0,0) + " " + (*velocity)(1,0) + " " + (*velocity)(2,0) + " " + "0.1 0.1 0.1");
	dGPSout->Flush();
	
}

String^ CExportGPSFile::GPSFileNameProvider(String^ m_inputDirectory)
{
	/*dGPSout = gcnew StreamWriter(m_GPSFileName);
	String^ GPSfilename = gcnew String("..\\..\\Integration\\Input\\GPS\\"+tempStr[3]->Remove(tempStr[3]->Length-4)+"_GPS.txt" );*/
	//StreamWriter^ dGPSout = gcnew StreamWriter(GPSfilename);

	array<Char>^sepDir = gcnew array<Char>{'/'};
	array<String^>^ tempStr;
	tempStr=m_inputDirectory->Split( sepDir ,StringSplitOptions::RemoveEmptyEntries);
	int fileNameOrder = tempStr->Length;
	String^ fileName = tempStr[fileNameOrder-1];
	String^ fileNameGPS = "..\\Output\\Simulated GPS\\" + fileName->Remove(fileName->Length-4) + "_GPS.txt";
	return fileNameGPS;
}

void CExportGPSFile::GPSFileOpen()
{
	String^ fileName=GPSFileNameProvider(m_inputDirectory);
	dGPSout = gcnew StreamWriter(fileName);
}

void CExportGPSFile::GPSFileClose()
{
	dGPSout->Close();
	delete dGPSout;
}