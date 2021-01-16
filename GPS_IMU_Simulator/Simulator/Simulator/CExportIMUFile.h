using namespace System;
using namespace std;
#include "CMatrix.h"
#include <iostream>
#include <fstream>
using namespace System::IO;

ref class CExportIMUFile
{
private:
	StreamWriter^ dIMUout;
	String^ m_inputDirectory;
public:
	CExportIMUFile(String^);
	~CExportIMUFile();

	void IMUFileOpen();
	void IMUFileClose();
	void WriteIMUFileContent(double , CMatrix* , CMatrix* );
	String^ IMUFileNameProvider(String^);

};