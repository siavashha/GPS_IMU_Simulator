#include "CProcessing.h"

CProcessing::CProcessing(String ^ trajectoryFileName)
{
	m_trajectoryFileName  = trajectoryFileName;
	readTrajectory = gcnew CReadTrajectory(m_trajectoryFileName);
	matrixShow =  gcnew CDisplayMatrixElements();
	carPosVel=  gcnew CPosVelCalc();
	carVelAcc=  gcnew CVelAccCalc();
	carOrientation =  gcnew COrientationCalc();
	carAngularRate = gcnew CAngularRateCalc();
	carAccelerometer = gcnew CErrorFreeAccelerometer();
	carGyro = gcnew CErrorFreeGyro();
	simulatedGPSOutput = gcnew CExportGPSFile(m_trajectoryFileName);
	simulatedIMUOutput = gcnew CExportIMUFile(m_trajectoryFileName);
	simIMUPos = gcnew CIMUProcessing();
	simulatedIMU = gcnew CIMUErrorGenerator(); 
	simulatedGPS = gcnew CGPSErrorGenerator();
	simulatedTrajectory = gcnew CExportTrajectory(m_trajectoryFileName);
	kalmanFilter = gcnew CKalmanFilter();
	errorCorrection = gcnew CEstimatedErrorCorrection();
}
CProcessing::~CProcessing()
{
	delete m_trajectoryFileName;
	delete readTrajectory;
	delete matrixShow;
	delete carPosVel;
	delete carVelAcc;
	delete carOrientation;
	delete carAngularRate;
	delete carAccelerometer;
	delete carGyro;
	delete simIMUPos;
	delete simulatedIMU;
	delete simulatedGPS;
	delete simulatedTrajectory;
	delete kalmanFilter;
	delete errorCorrection;
}

void CProcessing::Execute()
{
	CMatrix* carPosition = new CMatrix(3,1);
	CMatrix* carPositionGeodeticCoord = new CMatrix(3,1);
	CMatrix* carVelocity = new CMatrix(3,1);
	CMatrix* carAcceleration = new CMatrix(3,1);
	CMatrix* carEulerAngles = new CMatrix(3,1);
	CMatrix* carAngRate = new CMatrix(3,1);
	CMatrix* carFrontBumper = new CMatrix(3,1);
	CMatrix* carRearBumper = new CMatrix(3,1);
	CMatrix* carCenterPos = new CMatrix(3,1);
	CMatrix* inventory = new CMatrix(3,6);
	CMatrix* biasAccelerometer = new CMatrix(3,1);
	CMatrix* biasGyro = new CMatrix(3,1);
	CMatrix* scaleFactorErrorAccelerometer = new CMatrix(3,1);
	CMatrix* scaleFactorErrorGyro = new CMatrix(3,1);

	double time = 0;
	double timeVel = 0;
	double timeAcc = 0;
	double timeAngRate = 0;
	int epochNumber = 0;
	bool hasNextLine = false;

	matrixShow->MatrixReportOpen();
	simulatedGPSOutput->GPSFileOpen();
	simulatedIMUOutput->IMUFileOpen();
	simulatedTrajectory->TrajectoryFileOpen();
	do 
	{
		epochNumber++;
		hasNextLine = readTrajectory->ReadNextLine();
		readTrajectory->DataProvider();
		ReboundCARSIMParams(inventory , readTrajectory->bumper2bumperlength,readTrajectory->speed,readTrajectory->acceleration,readTrajectory->laneChangeIndicator1,readTrajectory->laneChangeIndicator2,readTrajectory->turnsign);
		//Calculates the car position, velocity, acceleration, Euler angles and angular rate in ENU frame
		this->local2ENU(*readTrajectory->carCenterMass,*carCenterPos);
		this->local2ENU(*readTrajectory->carFrontBumper,*carFrontBumper);
		this->local2ENU(*readTrajectory->carRearBumper,*carRearBumper);
		double time = readTrajectory->time;
		double timeVel = carPosVel->CarVelocityCalc(carCenterPos,time);
		(*carPosition) = (*carPosVel->carPreviousPosition);
		(*carVelocity) = (*carPosVel->carVelocity);
		double timeAcc  = carVelAcc->CarAccelerationCalc(carVelocity , timeVel  );
		(*carAcceleration) = (*carVelAcc->carAcceleration);
		carOrientation->CarOrientationCalc(carFrontBumper , carRearBumper );
		(*carEulerAngles) = (*carOrientation->carPitchRollYaw);
		double timeAngRate = carAngularRate->CarAngularRateCalc(carEulerAngles , time );
		(*carAngRate) = (*carAngularRate->carCurrentAngularRate);
		this->ENU2Geodetic(*carPositionGeodeticCoord , *carPosition);
		//Simulates the sensed acceleration and angular rate
		carAccelerometer->ErrorFreeAccelerometerCalc(carPositionGeodeticCoord,carVelAcc->carCurrentVelocity,carVelAcc->carAcceleration,carAngularRate->carPreviousEulerAngle);
		carGyro->ErrorFreeGyroCalc(carPositionGeodeticCoord,carVelAcc->carCurrentVelocity,carAngularRate->carPreviousEulerAngle,carAngularRate->carCurrentAngularRate);
		simulatedIMU->ErrorGenerator(this->IMUgrade,carAccelerometer->errorFreeAccelerometer,carGyro->errorFreeGyro);
		Console::WriteLine((*carPositionGeodeticCoord)(1,0));
		simulatedGPS->ErrorGenerator(this->GPStech , carPositionGeodeticCoord);

		if (epochNumber == 1)
		{
			CMatrix* Rb2n = new CMatrix(3,3);
			carOrientation->CarDirectCosineMatrixCalc(carAngularRate->carPreviousEulerAngle,Rb2n);
			double frontBumperLength = this->NormVector(*readTrajectory->carFrontBumper-*readTrajectory->carCenterMass);
			double rearBumperLength = this->NormVector( *readTrajectory->carRearBumper-*readTrajectory->carCenterMass);
			simulatedTrajectory->WriteFirstTrajectoryPoint(time, readTrajectory->carID, readTrajectory->carCenterMass, readTrajectory->carFrontBumper , readTrajectory->carRearBumper, readTrajectory->bumper2bumperlength , readTrajectory->speed, readTrajectory->acceleration , readTrajectory->laneChangeIndicator1 , readTrajectory->laneChangeIndicator2, readTrajectory->turnsign);			
			delete Rb2n;
		}
		//IMU initialization
		if (epochNumber == 3)
		{
			CMatrix* zeros = new CMatrix(3,1);
			CMatrix* Rb2n = new CMatrix(3,3);
			carOrientation->CarDirectCosineMatrixCalc(carAngularRate->carPreviousEulerAngle,Rb2n);
			simIMUPos->Initialization( carPositionGeodeticCoord , carVelAcc->carCurrentVelocity , Rb2n , zeros , zeros , zeros);
			delete Rb2n;
			delete zeros;
		}
		
		if (epochNumber>3)
		{
			simulatedGPSOutput->WriteGPSFileContent((*carPosVel->timeX)(0,0),simulatedGPS->simulatedGPSPos,carVelAcc->carCurrentVelocity);
			simulatedIMUOutput->WriteIMUFileContent((*carPosVel->timeX)(0,0),simulatedIMU->simulatedAccelerometer,simulatedIMU->simulatedGyro);
			double dT = (*carPosVel->timeX)(1,0)-(*carPosVel->timeX)(0,0);
			if ( (*carVelAcc->timeV)(0,0)>0)
			{
				/*CMatrix* testRb2n = new CMatrix(3,3);
				carOrientation->CarDirectCosineMatrixCalc(carAngularRate->carPreviousEulerAngle,testRb2n);
				CMatrix* testPosition =  new CMatrix(3,1);
				(*testPosition) = (*carPositionGeodeticCoord);
				simIMUPos->PositionCalc(simulatedIMU->simulatedAccelerometer , simulatedIMU->simulatedGyro , dT , testRb2n , testPosition);*/
				simIMUPos->PositionCalc(simulatedIMU->simulatedAccelerometer , simulatedIMU->simulatedGyro , dT);

				double frontBumperLength = this->NormVector(*readTrajectory->carFrontBumper-*readTrajectory->carCenterMass);
				double rearBumperLength = this->NormVector( *readTrajectory->carRearBumper-*readTrajectory->carCenterMass);
				kalmanFilter->TimeUpdate(simIMUPos->positionInn,simIMUPos->velocityInn,simIMUPos->Rb2n,simulatedIMU->simulatedAccelerometer ,simulatedIMU->simulatedGyro , this->IMUgrade ,dT);
				//kalmanFilter->ObservationUpdate(this->GPStech , simulatedGPS->simulatedGPSPos , simIMUPos->positionInn);
				//Console::WriteLine( "//Observation Update//" );
				//Console::WriteLine( (*kalmanFilter->x)(5,0) );
				//Console::WriteLine( (*kalmanFilter->x)(6,0) );
				//Console::WriteLine( );
				errorCorrection->ErrorCorrection(simIMUPos->positionInn, simIMUPos->velocityInn, simIMUPos->Rb2n , biasAccelerometer ,  biasGyro , scaleFactorErrorAccelerometer, kalmanFilter->x);
				//simIMUPos->Initialization( errorCorrection->updatedGeodeticPositionInn , errorCorrection->updatedVelocityInn , errorCorrection->updatedRb2n , errorCorrection->updatedBiasAccelerometer , errorCorrection->updatedBiasGyro , errorCorrection->updatedScaleFactorErrorAccelerometer );
				//kalmanFilter->x = new CMatrix(16,1);
				//simulatedTrajectory->WriteTrajectoryContent((*carPosVel->timeX)(0,0) , readTrajectory->carID , errorCorrection->updatedGeodeticPositionInn , errorCorrection->updatedRb2n , frontBumperLength , rearBumperLength , inventory);
				simulatedTrajectory->WriteTrajectoryContent((*carPosVel->timeX)(0,0) , readTrajectory->carID , simIMUPos->positionInn , simIMUPos->Rb2n , frontBumperLength , rearBumperLength , inventory);
			}
		}
	}	while (hasNextLine);
	simulatedGPSOutput->GPSFileClose();
	simulatedIMUOutput->IMUFileClose();
	simulatedTrajectory->TrajectoryFileClose();
	matrixShow->MatrixReportClose();

	delete carPosition;
	delete carPositionGeodeticCoord;
	delete carVelocity;
	delete carAcceleration;
	delete carEulerAngles;
	delete carAngRate;
	delete carFrontBumper;
	delete carRearBumper;
	delete carCenterPos;
}

void CProcessing::ReboundCARSIMParams(CMatrix* inventory, double carLength ,double speed , double acceleration, int laneChangeIndicator1 , int laneChangeIndicator2 ,  wchar_t turnsign  )
{
	(*inventory)(0,0) = (*inventory)(1,0);
	(*inventory)(1,0) = (*inventory)(2,0);
	(*inventory)(2,0) = speed;

	(*inventory)(0,1) = (*inventory)(1,1);
	(*inventory)(1,1) = (*inventory)(2,1);
	(*inventory)(2,1) = acceleration;

	(*inventory)(0,2) = (*inventory)(1,2);
	(*inventory)(1,2) = (*inventory)(2,2);
	(*inventory)(2,2) = laneChangeIndicator1;

	(*inventory)(0,3) = (*inventory)(1,3);
	(*inventory)(1,3) = (*inventory)(2,3);
	(*inventory)(2,3) = laneChangeIndicator2;

	(*inventory)(0,4) = (*inventory)(1,4);
	(*inventory)(1,4) = (*inventory)(2,4);
	(*inventory)(2,4) = turnsign;

	(*inventory)(0,5) = (*inventory)(1,5);
	(*inventory)(1,5) = (*inventory)(2,5);
	(*inventory)(2,5) = carLength;
}

	
