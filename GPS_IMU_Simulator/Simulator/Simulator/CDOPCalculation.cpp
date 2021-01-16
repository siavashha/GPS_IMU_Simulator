#include "CDOPCalculation.h"
#include "CConversion.h"

//returns: the mean HDOP
//Inputs: time, coordinates of rover, 0 as GPS HDOP and 1 as GPS+GLONASS HDOP

CDOPCalculation::CDOPCalculation(CMatrix* errorFreeGPS, int gradeGPS)
{
	geodeticCordGPS = new CMatrix(3,1);
	(*geodeticCordGPS) = (*errorFreeGPS);
	qualityGPSIndex = gradeGPS;

}

CDOPCalculation::~CDOPCalculation()
{
	delete geodeticCordGPS;
	//delete satCoordRow;
}

double CDOPCalculation::MeanHDOPestimator()//CMatrix& carPosition,int GPStype)
{
	int k = 0;
	double sum = 0;
	for (int time=0;time<=900;time=time+900)
	{//900 seconds is 15 minutes
	double dop = HDOPestimator(time);
	sum = sum + dop;
	k++;
	}
	return sum / k;
}

double CDOPCalculation::HDOPestimator(int time)
{
	//GPS
	IEnumerator ^ files1;
	String^ SVMrootDirectory1= "..\\SVM\\GPS\\";

	if (Directory::Exists(SVMrootDirectory1))
	{
		array<String^>^ fileEntries = Directory::GetFiles(SVMrootDirectory1+"maps\\");
		files1 = fileEntries->GetEnumerator();
	}
	String^ fileNameMap1 = gcnew String(SVMrootDirectory1 + "maps\\pg" + time + "_1" + ".txt" );
	String^ fileNameMap2 = gcnew String(SVMrootDirectory1 + "maps\\pg" + time + "_2" + ".txt" );
	String^	fileNameLegend1 = gcnew String(SVMrootDirectory1 + "legend\\LPG" + time + "_1" + ".txt" );
	String^	fileNameLegend2 = gcnew String(SVMrootDirectory1 + "legend\\LPG" + time + "_2" + ".txt" );
	String^	fileNameSatellite1 = gcnew String(SVMrootDirectory1 + "satellites\\PG" + time + "_1" + ".txt" );
	String^	fileNameSatellite2 = gcnew String(SVMrootDirectory1 + "satellites\\PG" + time + "_2" + ".txt" );

	CMatrix* satVisibility1 = new CMatrix(0,0);
	CMatrix* satVisibility2 = new CMatrix(0,0);
	CMatrix* satCoord1 = new CMatrix(0,0);
	CMatrix* satCoord2 = new CMatrix(0,0);

	Console::WriteLine(fileNameMap1);


	while (files1->MoveNext())
	{
		String^ m_fileName = safe_cast<String^> (files1->Current);
		if (m_fileName->Equals(fileNameMap1))
		{
			int visibilityCode = loadSatelliteVisibilityMap(fileNameMap1);
			SVMdecoder(visibilityCode, *satVisibility1 , fileNameLegend1);
			Satloader( *satCoord1 , fileNameSatellite1 );
		}
		if (m_fileName->Equals(fileNameMap2))
		{
			int visibilityCode = loadSatelliteVisibilityMap(fileNameMap2);
			SVMdecoder(visibilityCode, *satVisibility2 , fileNameLegend2);
			Satloader( *satCoord2 , fileNameSatellite2 );
			satCoord1->AppendMatrixInColumn(*satCoord2);
			satVisibility1->AppendMatrixInRow(*satVisibility2);
		}
	}
	if (qualityGPSIndex!=2)
	{
		//GLONASS
		IEnumerator ^ files2;
		CMatrix* satVisibility3 = new CMatrix(0,0);
		CMatrix* satVisibility4 = new CMatrix(0,0);
		CMatrix* satCoord3 = new CMatrix(0,0);
		CMatrix* satCoord4 = new CMatrix(0,0);
		String^ SVMrootDirectory2 = "..\\SVM\\GLONASS\\";
		String^	fileNameMap3 = gcnew String(SVMrootDirectory2 + "maps\\pr" + time + "_1" + ".txt" );
		String^	fileNameMap4 = gcnew String(SVMrootDirectory2 + "maps\\pr" + time + "_2" + ".txt" );
		String^	fileNameLegend3 = gcnew String(SVMrootDirectory2 + "legend\\LPR" + time + "_1" + ".txt" );
		String^	fileNameLegend4 = gcnew String(SVMrootDirectory2 + "legend\\LPR" + time + "_2" + ".txt" );
		String^	fileNameSatellite3 = gcnew String(SVMrootDirectory2 + "satellites\\PR" + time + "_1" + ".txt" );
		String^	fileNameSatellite4 = gcnew String(SVMrootDirectory2 + "satellites\\PR" + time + "_2" + ".txt" );

		if (Directory::Exists(SVMrootDirectory2))
		{
			array<String^>^ fileEntries = Directory::GetFiles(SVMrootDirectory2+"maps\\");
			files2 = fileEntries->GetEnumerator();
		}
		while (files2->MoveNext())
		{
			String^ m_fileName = safe_cast<String^> (files2->Current);
			if (m_fileName->Equals(fileNameMap3))
			{
				int visibilityCode = loadSatelliteVisibilityMap(fileNameMap3);
				SVMdecoder(visibilityCode, *satVisibility3 , fileNameLegend3);
				Satloader( *satCoord3 , fileNameSatellite3 );
				satCoord1->AppendMatrixInColumn(*satCoord3);
				satVisibility1->AppendMatrixInRow(*satVisibility3);
			}
			if (m_fileName->Equals(fileNameMap4))
			{
				int visibilityCode = loadSatelliteVisibilityMap(fileNameMap4);
				SVMdecoder(visibilityCode, *satVisibility4 , fileNameLegend4);
				Satloader( *satCoord4 , fileNameSatellite4 );
				satCoord1->AppendMatrixInColumn(*satCoord4);
				satVisibility1->AppendMatrixInRow(*satVisibility4);
			}
		}	
	}
	
	CMatrix* visibleSatellites = new CMatrix(0,0);
	satelliteSelection( *satCoord1 , *geodeticCordGPS , *satVisibility1 , *visibleSatellites);
	//ElevCalc(*satCoord1 ,  *geodeticCordGPS);

	//displayMatrix(*satCoord1);
	//displayMatrix(visibleSatellites);
	//Console::WriteLine(visibleSatellites->GetColumnNumber());
	//Console::WriteLine(visibleSatellites->GetRowNumber());
	//Console::WriteLine("//VS1//");

	double hdop = DOPCalc( *visibleSatellites , *geodeticCordGPS);
	return hdop;
}

void CDOPCalculation::satelliteSelection( CMatrix& satCoord , CMatrix& geodeticCordGPS , CMatrix& satVisibility , CMatrix& visibleSatellites)
{
	int u = 0;
	double elevation;
	CMatrix* satCoordColumn = new CMatrix(3,1) ;

	for (int i=0; i < satCoord.GetColumnNumber()-1; i++)
	{
		if (satVisibility(i,0)==1)
		{
			(*satCoordColumn)(0,0) = satCoord(0,i);
			(*satCoordColumn)(1,0) = satCoord(1,i);
			(*satCoordColumn)(2,0) = satCoord(2,i);
		
			elevation = ElevCalc(*satCoordColumn ,  geodeticCordGPS);
			if (elevation >0)
			{
				visibleSatellites.AppendMatrixInColumn(*satCoordColumn);
			}
		}
	}
	delete satCoordColumn;
}

void CDOPCalculation::Satloader( CMatrix& satCoord , String^ fileName )
{
	StreamReader^ din = gcnew StreamReader(fileName);
	String^ strCurrentLine;
	String^ strHeader;
	strHeader = din->ReadLine();

	array<String^>^ tempStr;
	array<Char>^sep = gcnew array<Char>{','};

	while ((strCurrentLine = din->ReadLine())!=nullptr) 
	{
		CMatrix* satCoordRow = new CMatrix(3,1);
		//strCurrentLine = din->ReadLine();
		tempStr=strCurrentLine->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
		String ^prnstr=tempStr[0]->Substring(3);
		int prn=Convert::ToInt32(prnstr);
		double satLongDeg=Convert::ToDouble(tempStr[1]);
		double satLatDeg=Convert::ToDouble(tempStr[2]);
		double satHeightFeet=Convert::ToDouble(tempStr[3]);

		(*satCoordRow)(0,0)=satLongDeg;
		(*satCoordRow)(1,0)=satLatDeg;
		(*satCoordRow)(2,0)=satHeightFeet;

		satCoord.AppendMatrixInColumn(*satCoordRow);
		delete satCoordRow;
	}
	//displayMatrix(satCoord);
}

void CDOPCalculation::SVMdecoder(int visibilityCode,  CMatrix& satVisibility ,String^ fileName)
{
	StreamReader^ din = gcnew StreamReader(fileName);
	String^ strHeader;
	strHeader = din->ReadLine();
	String^ strCurrentLine;

	array<String^>^ tempStr;
	array<Char>^ sep = gcnew array<Char>{','};
	CMatrix* data = new CMatrix(1,1);
	while ((strCurrentLine = din->ReadLine())!=nullptr) {
		//strCurrentLine = din->ReadLine();
		tempStr=strCurrentLine->Split( sep );
		if (visibilityCode==Convert::ToInt32(tempStr[1]))
		{
			for (int i=3;i<=tempStr->Length-1;i++)
			{
				(*data)(0,0)=Convert::ToInt32(tempStr[i]);
				//Console::WriteLine(satCoord(i-3,4));
				satVisibility.AppendMatrixInRow(*data);
			}
			//Console::WriteLine();
			break;
		}
	}
}

int CDOPCalculation::loadSatelliteVisibilityMap(String^ fileName)
{
	StreamReader^ dinMaps = gcnew StreamReader(fileName);
	String^ strNumberOfColumns;
	String^ strNumberOfRows;
	String^ strCornerX;
	String^ strCornerY;
	String^ strCellSize;
	String^ strNoData;
	String^ strCurrentLine;
	//header of Satellite Visibility Map
	array<String^>^ tempStr;
	array<Char>^sep = gcnew array<Char>{' '};
	strNumberOfColumns = dinMaps->ReadLine();//number of columns
	strNumberOfRows = dinMaps->ReadLine();//number of rows
	strCornerX = dinMaps->ReadLine();//xllcorner
	tempStr=strCornerX->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
	double xllcorner = Convert::ToDouble( tempStr[1] );
	strCornerY = dinMaps->ReadLine();//yllcorner
	tempStr=strCornerY->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
	double yllcorner = Convert::ToDouble( tempStr[1] );
	strCellSize = dinMaps->ReadLine();//cellsize
	tempStr=strCellSize->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
	double cellSize = Convert::ToDouble( tempStr[1] );
	strNoData = dinMaps->ReadLine();//nodata 
	tempStr=strNoData->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
	int noData = Convert::ToInt32( tempStr[1] );

	//body of Satellite Visibility Map
	int x=(int) floor(( (*geodeticCordGPS)(0,0)-xllcorner ) / cellSize);
	int y=(int) floor(( (*geodeticCordGPS)(1,0)-yllcorner) / cellSize);
	Console::WriteLine((*geodeticCordGPS)(1,0));
	for (int i=0 ; i <= y ; i++ ) 
	{
		strCurrentLine = dinMaps->ReadLine();
	}
	tempStr=strCurrentLine->Split( sep ,StringSplitOptions::RemoveEmptyEntries);
	int visibilityCode=Convert::ToInt32(tempStr[x]);
	return visibilityCode;
}

double CDOPCalculation::DOPCalc(CMatrix& satCoord , CMatrix& rovCoord)
{
	int NumberOfSatellites = satCoord.GetColumnNumber(); 
	CMatrix* designMatrixA = new CMatrix(4 , NumberOfSatellites);
	//Console::WriteLine(satCoord(0,2));
	for (int i=0 ; i < NumberOfSatellites - 1;i++)
	{
		double dX = satCoord(0,i) - rovCoord(0,0);
		double dY = satCoord(1,i) - rovCoord(1,0);
		double dZ = satCoord(2,i) - rovCoord(2,0);
		double dist=sqrt(dX*dX+dY*dY+dZ*dZ);
		double dXr=dX/dist;
		double dYr=dY/dist;
		double dZr=dZ/dist;
		(*designMatrixA)(0,i)=dXr;
		(*designMatrixA)(1,i)=dYr;
		(*designMatrixA)(2,i)=dZr;
		(*designMatrixA)(3,i)=-1;
	}
	CMatrix* auxilaryMatrix = new CMatrix(4,4);
	(*auxilaryMatrix) = designMatrixA->GetTranspose()*(*designMatrixA);
	auxilaryMatrix->GetInverse();

	double hdop=sqrt((*auxilaryMatrix)(0,0)+(*auxilaryMatrix)(1,1));	
	delete auxilaryMatrix;
	delete designMatrixA;
	return hdop;
}

double CDOPCalculation::ElevCalc(CMatrix& satCoord , CMatrix& rovCoord)
{
	double PI=3.14159265358979323846;
	CConversion^ conversion = gcnew CConversion();
	CMatrix* dENU = new CMatrix(3,1);
	conversion->Geodetic2ENU(satCoord - rovCoord , *dENU);
	double dX =	(*dENU)(0,0);
	double dY = (*dENU)(1,0);
	double dZ = (*dENU)(2,0);
	double dist=sqrt(dX*dX+dY*dY+dZ*dZ);
	//double dXr=dX/dist;
	//double dYr=dY/dist;
	//double dZr=dZ/dist;
	double LongOriginiInRadian = conversion->Degree2Rad(conversion->longOrigin);
	double LatOriginiInRadian = conversion->Degree2Rad(conversion->latOrigin);
	double north = -cos(LongOriginiInRadian)*sin(LatOriginiInRadian)*dX-sin(LongOriginiInRadian)*sin(LatOriginiInRadian)*dY+cos(LongOriginiInRadian)*dZ;
    double east = -sin(LongOriginiInRadian)*dX+cos(LongOriginiInRadian)*dY;
    double vertical = cos(LongOriginiInRadian)*cos(LatOriginiInRadian)*dX+sin(LongOriginiInRadian)*cos(LatOriginiInRadian)*dY+sin(LatOriginiInRadian)*dZ;

	// compute elevation angle
    double elevationinRad= -acos(vertical/dist)+PI/2;   
	//double elevationinRad= -acos(dZ/dist)+PI/2;
	double elevationinDeg=conversion->Radian2Deg(elevationinRad);     // degrees
	return elevationinDeg;
}

void CDOPCalculation::displayMatrix(CMatrix mat)
{
	int u=mat.GetRowNumber();
	int v=mat.GetColumnNumber();
	//Console::Write(" u= "+u);
	for (int i=0;i<=u-1;i++){
		for (int j=0;j<=v-1;j++){
			Console::Write(mat(i,j)+"  ");}
		Console::WriteLine();
		
		}
	Console::WriteLine();
}
