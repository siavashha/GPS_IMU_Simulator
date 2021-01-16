#include "CDisplayMatrixElements.h"

CDisplayMatrixElements::CDisplayMatrixElements()
{
}

CDisplayMatrixElements::~CDisplayMatrixElements()
{
}

void CDisplayMatrixElements::MatrixShow(double time , CMatrix mat)
{
	Console::WriteLine(time);
	int u=mat.GetRowNumber();
	int v=mat.GetColumnNumber();
	//Console::Write(" u= "+u);
	for (int i=0;i<=u-1;i++)
	{
		for (int j=0;j<=v-1;j++)
		{
			Console::Write(mat(i,j)+"  ");
		}
		Console::WriteLine();
		Console::WriteLine();
	}
	Console::WriteLine();
}

void CDisplayMatrixElements::MatrixReportOpen()
{
	dout = gcnew StreamWriter("test.txt");
}

void CDisplayMatrixElements::MatrixReportClose()
{
	dout->Close();
	delete dout;
}
void CDisplayMatrixElements::MatrixReportWrite(double timeMatrix , CMatrix matrix)
{
	double t=timeMatrix;
	dout->Write(t+"  ");
	int u=matrix.GetRowNumber();
	int v=matrix.GetColumnNumber();
	//Console::Write(" u= "+u);
	for (int i=0;i<=u-1;i++)
	{
		for (int j=0;j<=v-1;j++)
		{
			dout->Write(matrix(i,j)+"  ");
		}
		//dout->WriteLine();
	}
	dout->WriteLine();
	dout->Flush();
}

void CDisplayMatrixElements::MatrixReportWrite(double timeMatrix , CMatrix matrix1 , CMatrix matrix2)
{
	double t=timeMatrix;
	dout->Write(t+"  ");
	int u1=matrix1.GetRowNumber();
	int v1=matrix1.GetColumnNumber();
	//Console::Write(" u= "+u);
	for (int i=0;i<=u1-1;i++)
	{
		for (int j=0;j<=v1-1;j++)
		{
			dout->Write(matrix1(i,j)+"  ");
		}
		//dout->WriteLine();
	}
	dout->Write("      ");
	int u2=matrix2.GetRowNumber();
	int v2=matrix2.GetColumnNumber();
	for (int i=0;i<=u2-1;i++)
	{
		for (int j=0;j<=v2-1;j++)
		{
			dout->Write(matrix2(i,j)+"  ");
		}
		//dout->WriteLine();
	}
	dout->WriteLine();
	dout->Flush();
}