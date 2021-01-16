using namespace System;
#include "CMatrix.h"
#include "CAccVelCalc.h"
#include "CVelPosCalc.h"
#include <math.h>

ref class CIMUProcessing 
{
private:
	CMatrix* biasAcc;
	CMatrix* biasGyro;
	CMatrix* scaleFactorErrorAcc;
	CMatrix* previousAccelerationInnENU;
	//CMatrix* scaleFactorErrorGyro;
public:
	CMatrix* velocityInn;
	CMatrix*positionInn;
	CMatrix* Rb2n;
	CIMUProcessing();
	~CIMUProcessing();
	void PositionCalc(CMatrix* , CMatrix* , double );
	void UpdateQuaternion(CMatrix& , CMatrix& , CMatrix& );
	void Quaternion2R(CMatrix& , CMatrix& );
	void R2Quaternion(CMatrix& , CMatrix& );
	void StandardizeQuaternion(CMatrix& );
	void Initialization(CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix* , CMatrix*);
	void GetRb2LFromAttitude(CMatrix& , CMatrix& );
	void GetRe2L(double , double , CMatrix& );
	void GetRL2e(double , double , CMatrix& );
	void CalcCoriolisIne(CMatrix& ,  CMatrix& );
	void IMUErrorRemoval(CMatrix* , CMatrix*);


};