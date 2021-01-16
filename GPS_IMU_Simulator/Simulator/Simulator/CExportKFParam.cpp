#include "CExportKFParam.h"

CExportKFParamFile::CExportKFParamFile(String^ KFParamFileName){
	StreamWriter^ dKFParamout = gcnew StreamWriter(KFParamFileName);
}
CExportKFParamFile::~CExportKFParamFile(){
	dKFParamout->Close();
	delete dKFParamout;
}

void CExportKFParamFile::WriteKFParamFileContent(){
	
}
