#include <iostream>
#include "CFileManager.h"
#include "CProcessing.h"


using namespace System;

int main()
{	
	//Network Parameters including the offsets (local and Geodetic) , scale , Azimuth 
	CPreProcessing^ preProcessing = gcnew CPreProcessing();
	preProcessing->NetworkParamCalculator("config.txt");
	CConversion^ conversion = gcnew CConversion();
	
    // pre specified trajectory
	String ^ trajsDir=gcnew String("../../truth_Trajectory/");
	CFileManager^ fileMan=gcnew CFileManager(trajsDir);
	fileMan->LoadDirFiles();
	CProcessing^ processSimulator; 
	while(fileMan->GetNextFile())
		{
		processSimulator = gcnew CProcessing(fileMan->m_fileName);
		Console::WriteLine(fileMan->m_fileName);
		processSimulator->Execute();
		}
	system("PAUSE");
	return 0;
}
