#pragma once
//#include <stdio.h>
#include "CMatrix.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "CConversion.h"
using namespace System;
using namespace System::IO;
using namespace System::Collections;
//using namespace Drawing;
//using std;

ref class CDOPCalculation 
{
private:
	int qualityGPSIndex;
public:
	CMatrix* geodeticCordGPS;
	//CMatrix* satCoordRow;
	CDOPCalculation( CMatrix* , int );
	~CDOPCalculation();
	//returns the code indicating the visible satellites
	//inputs: rover coordinate , time and index 1 and 2
	int loadSatelliteVisibilityMap(String^ );
	//returns a (matrix) vector of possible visible satellites 
	//inputs the satellites coordinates matrix, rover position matrix
	double ElevCalc(CMatrix& , CMatrix& );
	//returns the HDOP
	//inputs: the satellite coordinates, rover coordinates
	double DOPCalc(CMatrix& , CMatrix& );
	//returns the mean HDOP
	double MeanHDOPestimator( );
	// returns the visable satellites
	// input: the visibility code, time , index , satellites coordinates
	void SVMdecoder(int , CMatrix& , String^);
	// returns the coordinates of the satellites
	void Satloader( CMatrix&  , String^  );
	//retruns visible satellites coordinates (removes invisible satellites)
	// input: satellite coordiantes, time and index
	//void VisibleSats(CMatrix& ,CMatrix&,int , double , int,double , double);
	void displayMatrix(CMatrix );

	double HDOPestimator(int );

	void satelliteSelection( CMatrix&  , CMatrix&  , CMatrix&  , CMatrix& );


};

