#include "CGPSErrorGenerator.h"

CGPSErrorGenerator::CGPSErrorGenerator()
{
	srand( (unsigned)time( NULL ) );
	simulatedGPSPos = new CMatrix(3,1);
}

CGPSErrorGenerator::~CGPSErrorGenerator()
{	
	delete simulatedGPSPos;
}

double CGPSErrorGenerator::StandardNormalGenerator()
{
	// generate random number with uniform distribution
	double PI=3.14159265358979323846;
	double var1;
	double var2;
	
	do 
	{
		var1 =  ( double(rand()) / double(RAND_MAX) ) ;
		var2 =  ( double(rand()) / double(RAND_MAX) ) ;
	} while (!(( var1 > 0 && var1 <= 1.0 ) && ( var2 > 0 && var2 <= 1.0 )));

    double R=sqrt( -2 * log(var1));
	double theta = 2  * var2 * PI;

    //        calculate Standard Normal Distribution
    if (R*cos(theta)<1e10)
	return R*cos(theta);
	else if (R*sin(theta)<1e10)
	return R*sin(theta);
	else
	return 0;
}

void CGPSErrorGenerator::ErrorGenerator(int gradeGPS, CMatrix* errorFreeGPS)
{
	/*double ephemeridesBias = 0;
	double ephemeridesRandom = 0;
	double satClockBias = 0;
	double satClockRandom = 0;
	double ionosphereBias = 0;
	double ionosphereRandom = 0;
	double troposphereBias = 0;
	double troposphereRandom = 0;
	double multipathBias = 0;
	double multipathRandom = 0;
	double recBias = 0;
	double recRandom = 0;*/
	double error = 0;
	if (gradeGPS==3) //GPS Standalone
	{
		/*ephemeridesBias=2.1;
		ephemeridesRandom=0.0;
		satClockBias=2.0;
		satClockRandom=0.7;
		ionosphereBias=4.0;
		ionosphereRandom=0.5;
		troposphereBias=0.5;
		troposphereRandom=0.5;
		multipathBias=1.0;
		multipathRandom=1.0;
		recBias=0.5;
		recRandom=0.2;*/
		error = 6 * StandardNormalGenerator();
		}
	else if(gradeGPS==2) //WAAS
	{
		//ephemeridesBias=0.0;
		//ephemeridesRandom=0.0;
		//satClockBias=0.0;
		//satClockRandom=0.0;
		//ionosphereBias=2.0;
		//ionosphereRandom=0.5;
		//troposphereBias=0.5;
		//troposphereRandom=0.5;
		//multipathBias=1.0;
		//multipathRandom=1.0;
		//recBias=0.5;
		//recRandom=0.2;
		error = 1 * StandardNormalGenerator();
		}
	else if(gradeGPS==1)//RTK
	{
		//ephemeridesBias=0.0;
		//ephemeridesRandom=0.0;
		//satClockBias=0.0;
		//satClockRandom=0.0;
		//ionosphereBias=0.15;
		//ionosphereRandom=0.05;
		//troposphereBias=0.10;
		//troposphereRandom=0.05;
		//multipathBias=0.0;
		//multipathRandom=0.3;
		//recBias=0.05;
		//recRandom=0.02;
		error = 0.15 * StandardNormalGenerator();
		}
		else if(gradeGPS==0) //Error-Free GPS
		{
		//ephemeridesBias=0.0;
		//ephemeridesRandom=0.0;
		//satClockBias=0.0;
		//satClockRandom=0.0;
		//ionosphereBias=0.0;
		//ionosphereRandom=0.0;
		//troposphereBias=0.0;
		//troposphereRandom=0.0;
		//multipathBias=0.0;
		//multipathRandom=0.0;
		//recBias=0.0;
		//recRandom=0.0;
		error = 0 * StandardNormalGenerator();
		}

	/*double randVarSatClock=StandardNormalGenerator();
	double randVarIono=StandardNormalGenerator();
	double randVarTropo=StandardNormalGenerator();
	double randVarMulti=StandardNormalGenerator();
	double randVarNoise=StandardNormalGenerator();
	double uere=ephemeridesBias+satClockBias+ionosphereBias+troposphereBias+multipathBias+recBias+satClockRandom*randVarSatClock+ionosphereRandom*randVarIono+troposphereRandom*randVarTropo+multipathRandom*randVarMulti+recRandom*randVarNoise;
	//double error=uere*hdop;
	//double b=(*errorFreeGPS)(0,0);
	//double j=((error)/earth_R)*180/PI;/*/
	Console::WriteLine((*errorFreeGPS)(1,0));
	CDOPCalculation^ hDOP = gcnew CDOPCalculation(errorFreeGPS,gradeGPS);
	double calculatedHDOP = hDOP->MeanHDOPestimator();
	double PI=3.14159265358979323846;
	double earth_R = 6378131.0;
	error = calculatedHDOP * error;
	
	(*simulatedGPSPos)(0,0)=(*errorFreeGPS)(0,0)+((error)/earth_R)*180/PI;
	(*simulatedGPSPos)(1,0)=(*errorFreeGPS)(1,0)+((error)/earth_R)*180/PI;
	(*simulatedGPSPos)(2,0)=(*errorFreeGPS)(2,0)+(error*1.5);
}