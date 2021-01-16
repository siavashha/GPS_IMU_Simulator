#include "CMatrix.h"
#include "CConversion.h"
#include "CReadTrajectory.h"
#include "CPosVelCalc.h"
#include "CVelAccCalc.h"
#include "COrientationCalc.h"
#include "CAngularRateCalc.h"
#include "CErrorFreeAccelerometer.h"
#include "CErrorFreeGyro.h"
#include "CExportGPSFile.h"
#include "CExportIMUFile.h"
#include "CIMUProcessing.h"
#include "CIMUErrorGenerator.h"
#include "CGPSErrorGenerator.h"
#include "CExportTrajectory.h"
#include "CKalmanFilter.h"
#include "CEstimatedErrorCorrection.h"
#include <math.h>

ref class CProcessing : public CConversion 
{
private:
	String ^ m_trajectoryFileName;
	CExportGPSFile^ simulatedGPSOutput;
	CExportIMUFile^ simulatedIMUOutput;
	CReadTrajectory^ readTrajectory;
	CDisplayMatrixElements^ matrixShow;
	CPosVelCalc^ carPosVel;
	CVelAccCalc^ carVelAcc;
	COrientationCalc^ carOrientation;
	CAngularRateCalc^ carAngularRate;
	CErrorFreeAccelerometer^ carAccelerometer;
	CErrorFreeGyro^ carGyro;
	CIMUProcessing^ simIMUPos;
	CIMUErrorGenerator^ simulatedIMU; 
	CGPSErrorGenerator^ simulatedGPS;
	CExportTrajectory^ simulatedTrajectory;
	CKalmanFilter^ kalmanFilter;
	CEstimatedErrorCorrection^ errorCorrection; 
public:
	CProcessing(String ^);
	~CProcessing();
	void Execute();
	void ReboundCARSIMParams(CMatrix* , double , double , double , int , int ,  wchar_t);

};