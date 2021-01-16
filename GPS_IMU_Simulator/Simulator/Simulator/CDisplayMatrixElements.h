#include "CMatrix.h"
#include <time.h>
#include <iostream>
#include <fstream>
using namespace System::IO;
using namespace System;
using namespace std;

ref class CDisplayMatrixElements 
{
private:
	StreamWriter^ dout;
public:
	CDisplayMatrixElements();
	~CDisplayMatrixElements();
	void MatrixShow(double,CMatrix );
	void MatrixReportOpen();
	void MatrixReportClose();
	void MatrixReportWrite( double , CMatrix);
	void MatrixReportWrite( double , CMatrix,CMatrix);
};