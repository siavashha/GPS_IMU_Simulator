using namespace System;
using namespace std;
#include <iostream>
#include <fstream>
using namespace System::IO;

ref class CExportKFParamFile
{
private:
	StreamWriter^ dKFParamout;
public:
	CExportKFParamFile(String^);
	~CExportKFParamFile();
	void WriteKFParamFileContent(  );

};