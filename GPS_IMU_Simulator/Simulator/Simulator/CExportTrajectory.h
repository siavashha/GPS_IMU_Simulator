using namespace System;
using namespace std;
#include "CMatrix.h"
#include <iostream>
#include <fstream>
using namespace System::IO;

ref class CExportTrajectory
{
private:
	StreamWriter^ dout;
	String^ m_inputDirectory;
public:
	CExportTrajectory(String^);
	~CExportTrajectory();

	void TrajectoryFileOpen();
	void TrajectoryFileClose();
	void WriteTrajectoryContent(double , int , CMatrix* , CMatrix* , double , double , CMatrix* );
	void WriteFirstTrajectoryPoint(double , int , CMatrix* , CMatrix* , CMatrix*  , double , double , double , int , int , wchar_t );
	String^ TrajectoryFileNameProvider(String^);

};