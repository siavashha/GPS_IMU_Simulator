using namespace System;
using namespace std;
#include "CMatrix.h"
#include <iostream>
#include <fstream>
using namespace System::IO;

ref class CExportGPSFile
{
private:
	StreamWriter^ dGPSout;
	String^ m_inputDirectory;
public:
	CExportGPSFile(String^);
	~CExportGPSFile();
	void GPSFileOpen();
	void GPSFileClose();
	void WriteGPSFileContent(double , CMatrix* , CMatrix* );
	String^ GPSFileNameProvider(String^);
};