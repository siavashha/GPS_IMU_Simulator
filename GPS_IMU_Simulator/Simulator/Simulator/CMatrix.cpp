//#include "stdafx.h"
#include "CMatrix.h"
#include <math.h>
#include <memory.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
//Overloading the constructor
//whitout argument: zero matrix 
CMatrix::CMatrix()
{
	Initate();
}
//with the value of the matrix
CMatrix::CMatrix(const CMatrix& OriginalMatrix)
{
	Initate();
	InitMatrix(OriginalMatrix.GetDataAddress(), OriginalMatrix.GetRowNumber(), OriginalMatrix.GetColumnNumber());
}
//  with the vector data and number of rows and columns
//      nRowNumber: number of rows
//      nColumnNumber: number of columns
CMatrix::CMatrix(const f8* pData, int4 i4RowNumber, int4 i4ColumnNumber)
{
	Initate();
	InitMatrix(pData, i4RowNumber, i4ColumnNumber);
}
//matrix of zeros with number of rows and columns
CMatrix::CMatrix(int4 i4RowNumber, int4 i4ColumnNumber)
{
	m_pData = new f8[ i4RowNumber * i4ColumnNumber ];
	if(m_pData != NULL )
	{
		memset(m_pData, 0, sizeof(f8) * i4RowNumber * i4ColumnNumber);
		m_i4RowNumber = i4RowNumber;
		m_i4ColumnNumber = i4ColumnNumber;
	}
	else
	{
		m_i4RowNumber = 0;
		m_i4ColumnNumber = 0;
	}
}

CMatrix::~CMatrix()
{
	delete [] m_pData;
	m_pData = NULL;
}

void CMatrix::Initate()
{
	m_pData			 = NULL;
	m_i4RowNumber	 = 0;
	m_i4ColumnNumber = 0;
}

// The data with the number of rows and columns
//////////////////////////////////////////////////////////////////////
int4 CMatrix::InitMatrix(const f8* pData, int4 i4RowNumber, int4 i4ColumnNumber)
{
	if(i4RowNumber<0 || i4ColumnNumber<0) return 0;
	if(NULL==pData && 0==i4RowNumber && 0==i4ColumnNumber)
	{
		if(NULL != m_pData) 
		{
			delete []m_pData;
			m_pData = NULL;
		}
		m_i4RowNumber	 = 0;
		m_i4ColumnNumber = 0;
		return 1;
	}
	if(m_i4RowNumber * m_i4ColumnNumber != i4RowNumber * i4ColumnNumber)
	{
		delete []m_pData;
		m_pData = new f8[ i4RowNumber * i4ColumnNumber ];
		if(NULL == m_pData)
		{
			m_i4RowNumber = 0;
			m_i4ColumnNumber = 0;
			return 0;
		}
	}
	memcpy(m_pData, pData, sizeof(f8) * i4RowNumber * i4ColumnNumber );
	m_i4RowNumber = i4RowNumber;
	m_i4ColumnNumber = i4ColumnNumber;
	return 1;
}


//constructing zero matrix

int4 CMatrix::InitZeroMatrix(int4 i4RowNumber,int4 i4ColumnNumber)
{
	if(i4RowNumber<=0 || i4ColumnNumber<=0) return 0;

	if(m_i4RowNumber * m_i4ColumnNumber != i4RowNumber * i4ColumnNumber)
	{
		delete []m_pData;
		m_pData = new f8[ i4RowNumber * i4ColumnNumber ];
		if(NULL == m_pData)
		{
			m_i4RowNumber = 0;
			m_i4ColumnNumber = 0;
			return 0;
		}
	}
	m_i4RowNumber	= i4RowNumber;
	m_i4ColumnNumber = i4ColumnNumber;
	memset(m_pData, 0, sizeof(f8) * i4RowNumber * i4ColumnNumber);

	return 1;
}

//deleting the matrix
int4 CMatrix::Dump()
{
	if(NULL != m_pData) 
	{
		delete [] m_pData;//和原有数据断绝关系
		m_pData = NULL;
	}
	m_i4RowNumber    = 0;
	m_i4ColumnNumber = 0;
	return 1;
}

//getting data and saving it in predefined matrix
int4 CMatrix::GetData(f8* pDestiData) const
{
	memcpy(pDestiData, m_pData, sizeof(f8) * m_i4RowNumber * m_i4ColumnNumber);
	return 1;
}

//getting the address of matrix
f8* CMatrix::GetDataAddress() const
{ 
	return m_pData; 
}

//inverse of the matrix
int4 CMatrix::GetInverse(CMatrix& maInverse)const
{
	if( m_i4RowNumber != m_i4ColumnNumber) return 0;

	maInverse = *this;
	if(0 == this->InversMatrix(maInverse.GetDataAddress(), m_i4RowNumber))
	{
		maInverse.Dump();
		return 0;
	}
	return 1;
	
}

//inverse of the matrix
CMatrix CMatrix::GetInverse()const
{
	CMatrix TmpMatrix;
	if( m_i4RowNumber != m_i4ColumnNumber) return TmpMatrix;

	TmpMatrix = *this;
	if(0 == this->InversMatrix(TmpMatrix.GetDataAddress(), m_i4RowNumber))
	{
		TmpMatrix.Dump();
	}
	
	return TmpMatrix;
}

//inverse of matrix
int4 CMatrix::Inverse()
{
	if(m_i4RowNumber != m_i4ColumnNumber) return 0;
	return this->InversMatrix(m_pData, m_i4RowNumber);
}

//Transpose of the matrix
int4 CMatrix::GetTranspose(CMatrix& maTransposed)const
{
	if(1 != maTransposed.InitZeroMatrix(m_i4ColumnNumber,m_i4RowNumber)) return 0;
	Transpose(m_pData,maTransposed.GetDataAddress(),m_i4RowNumber,m_i4ColumnNumber);
	return 1;
}

//Transpose of the matrix
CMatrix CMatrix::GetTranspose()const
{
	CMatrix TmpMatrix;	
	if(1 != TmpMatrix.InitZeroMatrix(m_i4ColumnNumber,m_i4RowNumber)) return TmpMatrix;
	Transpose(m_pData,TmpMatrix.GetDataAddress(),m_i4RowNumber,m_i4ColumnNumber);
	
	return TmpMatrix;
}

//Transpose of the matrix
int4 CMatrix::Transpose()
{
	f8* pf8Tmp = new f8[m_i4RowNumber * m_i4ColumnNumber];
	if(NULL == pf8Tmp) return 0;
	Transpose(m_pData, pf8Tmp, m_i4RowNumber, m_i4ColumnNumber);
	delete []m_pData;
	m_pData = pf8Tmp;

	int4 i4Tmp = m_i4RowNumber;
	m_i4RowNumber = m_i4ColumnNumber;
	m_i4ColumnNumber = i4Tmp;

	return 1;
}

//Orthogonilize the matrix    (inv(M')+M)/2
int4 CMatrix::Orthogonalize()
{
	if(m_i4RowNumber != m_i4ColumnNumber) return 0;
	
	CMatrix ma1,ma2=*this;
	int4 i=0;
	for(;;)
	{
		ma1 = ( (ma2.GetTranspose()).GetInverse() + ma2) / 2;
		if((ma1-ma2).GetNorm()<1.0e-8) break;
		ma2 = ma1;
		if(i>100) return 0;
	}
	*this = ma1;
	
	return 1;
	
}

//get subset of the matrix from first row and column to SubRowNumber and SubColumnNumber
int4 CMatrix::GetLeftTopSubMatrix(int4 i4SubRowNumber, int4 i4SubColumnNumber, CMatrix& maSubMatrix)const
{
	if(i4SubRowNumber>m_i4RowNumber || i4SubColumnNumber>m_i4ColumnNumber) return 0;
	
	if(1 != maSubMatrix.InitZeroMatrix(i4SubRowNumber, i4SubColumnNumber)) return 0;
	for(int4 i=0; i<i4SubRowNumber; i++)
	{
		memcpy(maSubMatrix.GetDataAddress() + i * i4SubColumnNumber, m_pData + i * m_i4ColumnNumber, i4SubColumnNumber * sizeof(f8));
	}
	
	return 1;
}

//get subset of the matrix from first row and column to SubRowNumber and SubColumnNumber
CMatrix CMatrix::GetLeftTopSubMatrix(int4 i4SubRowNumber,int4 i4SubColumnNumber)const
{
	CMatrix maResult;
	if(i4SubRowNumber>m_i4RowNumber || i4SubColumnNumber>m_i4ColumnNumber) return maResult;
	
	if(1 != maResult.InitZeroMatrix(i4SubRowNumber, i4SubColumnNumber)) return maResult;
	for(int4 i=0; i<i4SubRowNumber; i++)
	{
		memcpy(maResult.GetDataAddress() + i * i4SubColumnNumber, m_pData + i * m_i4ColumnNumber, i4SubColumnNumber * sizeof(f8));
	}
	
	return maResult;
}

//get subset of the matrix from first BeginRowNo and BeginColumnNo to SubRowNumber and SubColumnNumber
int4 CMatrix::GetSubMatrix(int4 i4BeginRowNo, int4 i4SubRowNumber, int4 i4BeginColumnNo, int4 i4SubColumnNumber, CMatrix& maSubMatrix)const
{
	if(i4BeginRowNo + i4SubRowNumber > m_i4RowNumber
		|| i4BeginColumnNo + i4SubColumnNumber > m_i4ColumnNumber) return 0;
	
	if(1 != maSubMatrix.InitZeroMatrix(i4SubRowNumber, i4SubColumnNumber)) return 0;	
	for(int4 i=0; i<i4SubRowNumber; i++)
	{
		memcpy(maSubMatrix.GetDataAddress() + i * i4SubColumnNumber, m_pData + (i4BeginRowNo+i) * m_i4ColumnNumber + i4BeginColumnNo, i4SubColumnNumber * sizeof(f8));
	}
	
	return 1;	
}

//get subset of the matrix from first BeginRowNo and BeginColumnNo to SubRowNumber and SubColumnNumber
CMatrix CMatrix::GetSubMatrix(int4 i4BeginRowNo, int4 i4SubRowNumber, int4 i4BeginColumnNo, int4 i4SubColumnNumber)const
{
	CMatrix maResult;
	if(i4BeginRowNo + i4SubRowNumber > m_i4RowNumber
		|| i4BeginColumnNo + i4SubColumnNumber > m_i4ColumnNumber) return maResult;
	
	if(1 != maResult.InitZeroMatrix(i4SubRowNumber, i4SubColumnNumber)) return maResult;	
	for(int4 i=0; i<i4SubRowNumber; i++)
	{
		memcpy(maResult.GetDataAddress() + i * i4SubColumnNumber, m_pData + (i4BeginRowNo+i) * m_i4ColumnNumber + i4BeginColumnNo, i4SubColumnNumber * sizeof(f8));
	}
	
	return maResult;
}


//adding two matrices
int4 CMatrix::AddMatrix(CMatrix& maRight, CMatrix& maResult)const
{
	if((m_i4RowNumber != maRight.m_i4RowNumber) || (m_i4ColumnNumber != maRight.m_i4ColumnNumber)) return 0;

	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	if(1 != maResult.InitMatrix(m_pData, m_i4RowNumber, m_i4ColumnNumber)) return 0;

	for(int4 i=0; i<i4ElementNumber; i++)
	{
		maResult.m_pData[i] += maRight.m_pData[i];
	}
	
	return 1;
}

//gives the submatrix
int4 CMatrix::SubMatrix(CMatrix& maRight, CMatrix& maResult)const
{
	if((m_i4RowNumber != maRight.m_i4RowNumber) || (m_i4ColumnNumber != maRight.m_i4ColumnNumber)) return 0;
	
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	if(1 != maResult.InitMatrix(m_pData, m_i4RowNumber, m_i4ColumnNumber)) return 0;
	
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		maResult.m_pData[i] -= maRight.m_pData[i];
	}
	
	return 1;
}

//multiplication of two matrices
int4 CMatrix::MultiplyMatrix(CMatrix& maRight, CMatrix& maResult)const
{
	if(m_i4ColumnNumber != maRight.m_i4RowNumber) return 0;//A阵列数不等于B阵行数
	if(1 != maResult.InitZeroMatrix(m_i4RowNumber, maRight.m_i4ColumnNumber)) return 0;
	Mult(m_pData, maRight.m_pData, maResult.m_pData, m_i4RowNumber, m_i4ColumnNumber, maRight.m_i4ColumnNumber);
	
	return 1;
}

//gives one element of the matrix 
int4 CMatrix::GetElement(int4 i4RowNo, int4 i4ColumnNo, f8& f8Element)const
{
	if(i4RowNo >= m_i4RowNumber || i4ColumnNo >= m_i4ColumnNumber) return 0;
	f8Element = *(m_pData + i4RowNo * m_i4ColumnNumber + i4ColumnNo);
	
	return 1;
}

//gives one element of the matrix 
f8 CMatrix::GetElement(int4 i4RowNo, int4 i4ColumnNo)const
{
	if(i4RowNo >= m_i4RowNumber || i4ColumnNo >= m_i4ColumnNumber) return INFINITY;
	return *(m_pData + i4RowNo * m_i4ColumnNumber + i4ColumnNo);
}

//changes one element of the matrix 
int4 CMatrix::ChangeElement(int4 i4RowNo, int4 i4ColumnNo, f8 f8Element)const
{
	if(i4RowNo >= m_i4RowNumber || i4ColumnNo >= m_i4ColumnNumber) return 0;
	*(m_pData + i4RowNo * m_i4ColumnNumber + i4ColumnNo) = f8Element;

	return 1;
}


int4 CMatrix::ExChangeTwoRows(int4 i4RowNo1, int4 i4RowNo2)const
{
	if(i4RowNo1 >= m_i4RowNumber || i4RowNo2 >= m_i4RowNumber) return 0;
	if(i4RowNo1 == i4RowNo2) return 1;
	f8* pTmp = new f8[m_i4ColumnNumber];
	if(NULL == pTmp) return 0;
	memcpy(pTmp, m_pData+i4RowNo1*m_i4ColumnNumber, sizeof(f8)*m_i4ColumnNumber);
	memcpy(m_pData+i4RowNo1*m_i4ColumnNumber, m_pData+i4RowNo2*m_i4ColumnNumber, sizeof(f8)*m_i4ColumnNumber);
	memcpy(m_pData+i4RowNo2*m_i4ColumnNumber, pTmp, sizeof(f8)*m_i4ColumnNumber);
	delete []pTmp;
	return 1;
}

int4 CMatrix::ExChangeTwoColumns(int4 i4ColumnNo1, int4 i4ColumnNo2)const
{
	if(i4ColumnNo1 >= m_i4ColumnNumber || i4ColumnNo2 >= m_i4ColumnNumber) return 0;
	if(i4ColumnNo1 == i4ColumnNo2) return 1;
	
	f8 f8Tmp = 0.0;
	for(int4 i=0; i<m_i4RowNumber; i++)
	{
		f8Tmp = m_pData[i*m_i4RowNumber+i4ColumnNo1];
		m_pData[i*m_i4RowNumber+i4ColumnNo1] = m_pData[i*m_i4RowNumber+i4ColumnNo2];
		m_pData[i*m_i4RowNumber+i4ColumnNo2] = f8Tmp;
	}
	return 1;
}

int4 CMatrix::ChangeSubMatrix(int4 i4BeginRowNo, int4 i4BeginColumnNo, const CMatrix& maSecMatrix)// 改变子块
{
	int4 i4SecRow = maSecMatrix.GetRowNumber();
	int4 i4SecColumn = maSecMatrix.GetColumnNumber();
	if((i4BeginRowNo+i4SecRow) > m_i4RowNumber || (i4BeginColumnNo+i4SecColumn) > m_i4ColumnNumber) return 0;
	f8 f8Data = 0;
	for(int i= 0; i<i4SecRow; i++)
		for(int j=0; j<i4SecColumn; j++)
		{
			ChangeElement(i4BeginRowNo+i, i4BeginColumnNo+j, *(maSecMatrix.GetDataAddress()+ i * i4SecColumn + j));//改变矩阵元素
		}

	return 1;
}

int4 CMatrix::AddSubMatrixMember(int4 i4BeginRowNo, int4 i4BeginColumnNo, const CMatrix& maSecMatrix)// 子块元素相加，改变原有矩阵元素
{
	int4 i4SecRow = maSecMatrix.GetRowNumber();
	int4 i4SecColumn = maSecMatrix.GetColumnNumber();
	if((i4BeginRowNo + i4SecRow) > m_i4RowNumber || (i4BeginColumnNo + i4SecColumn) > m_i4ColumnNumber) return 0;
	for(int i= 0; i<i4SecRow; i++)
		for(int j=0; j<i4SecColumn; j++)
		{
			*(m_pData + (i4BeginRowNo+i) * m_i4ColumnNumber + i4BeginColumnNo+j) += *(maSecMatrix.GetDataAddress()+ i * i4SecColumn + j);
		}
	return 1;
}

int4 CMatrix::ChangeOneRowWithMember(int4 i4RowNo, f8 f8Element)
{
	if(i4RowNo >= m_i4RowNumber) return 0;
	for(int4 i=0; i<m_i4ColumnNumber; i++)
	{
		m_pData[i4RowNo * m_i4ColumnNumber + i] = f8Element;
	}
	return 1;
}

int4 CMatrix::ChangeOneColumnWithMember(int4 i4ColumnNo, f8 f8Element)
{
	if(i4ColumnNo >= m_i4ColumnNumber) return 0;
	for(int4 i=0; i<m_i4RowNumber; i++)
	{
		m_pData[i * m_i4ColumnNumber + i4ColumnNo] = f8Element;
	}
	return 1;
}

int4 CMatrix::InsertNewRow(int4 i4RowNo, f8* pData, int4 i4DataNumber)
{
	if(NULL == pData) return 0;

	CMatrix mTmpMatrix(pData, 1, i4DataNumber);
	if(0 == mTmpMatrix.GetRowNumber()) return 0;//防止对象构造失败
	return InsertMatrixInRow(i4RowNo, mTmpMatrix);
}

int4 CMatrix::InsertNewColumn(int4 i4ColumnNo, f8* pData, int4 i4DataNumber)
{
	if(NULL == pData) return 0;
	
	CMatrix mTmpMatrix(pData, i4DataNumber, 1);
	if(0 == mTmpMatrix.GetRowNumber()) return 0;//防止对象构造失败
	return InsertMatrixInColomn(i4ColumnNo, mTmpMatrix);
}

int4 CMatrix::InsertMatrixInRow(int4 i4RowNo, const CMatrix& SecMatrix)
{
	if(0 == m_i4RowNumber * m_i4ColumnNumber)
	{
		if(i4RowNo != 0) return 0;
		m_pData = new f8[SecMatrix.GetRowNumber() * SecMatrix.GetColumnNumber()];
		if(NULL == m_pData) return 0;
		m_i4ColumnNumber = SecMatrix.GetColumnNumber();
		m_i4RowNumber = SecMatrix.GetRowNumber();
		memcpy(m_pData, SecMatrix.GetDataAddress(), m_i4ColumnNumber*m_i4RowNumber*sizeof(f8));
		return 1;
	}
	
	if ( (SecMatrix.GetColumnNumber() != m_i4ColumnNumber)
		|| i4RowNo < 0
		|| (i4RowNo > m_i4RowNumber)) 
		return 0;

	f8* pTmp = new f8[m_i4ColumnNumber * (m_i4RowNumber + SecMatrix.GetRowNumber())];
	if(NULL == pTmp) return 0;
	memcpy(pTmp, m_pData, i4RowNo*m_i4ColumnNumber*sizeof(f8));
	memcpy(pTmp + (i4RowNo)*m_i4ColumnNumber, SecMatrix.GetDataAddress(), SecMatrix.GetRowNumber()*m_i4ColumnNumber*sizeof(f8));

	if(m_i4RowNumber != i4RowNo)
		memcpy(pTmp + (i4RowNo)*m_i4ColumnNumber + SecMatrix.GetRowNumber()*m_i4ColumnNumber, m_pData+i4RowNo*m_i4ColumnNumber, 
			  (m_i4RowNumber - i4RowNo)*m_i4ColumnNumber*sizeof(f8));

	delete []m_pData;
	m_pData = pTmp;
	m_i4RowNumber += SecMatrix.GetRowNumber();

	return 1;
}

int4 CMatrix::InsertMatrixInColomn(int4 i4ColumnNo, const CMatrix& SecMatrix)
{

	if(0 == m_i4RowNumber * m_i4ColumnNumber)
	{
		if(i4ColumnNo != 0)	return 0;
		m_pData = new f8[SecMatrix.GetRowNumber() * SecMatrix.GetColumnNumber()];
		if(NULL == m_pData) return 0;
		m_i4RowNumber = SecMatrix.GetRowNumber();
		m_i4ColumnNumber = SecMatrix.GetColumnNumber();
		memcpy(m_pData, SecMatrix.GetDataAddress(), m_i4ColumnNumber*m_i4RowNumber*sizeof(f8));
		return 1;
	}
	
	if ( (SecMatrix.GetRowNumber()!=m_i4RowNumber)
		|| i4ColumnNo<0 
		|| (i4ColumnNo > m_i4ColumnNumber) ) 
		return 0;

	int4 i4SecColNum = SecMatrix.GetColumnNumber();
	f8* pTmp = new f8[m_i4RowNumber * (m_i4ColumnNumber + i4SecColNum)];
	if(NULL == pTmp)	return 0;

	f8* pSecMatrix = SecMatrix.GetDataAddress();
	for(int4 i=0; i<m_i4RowNumber; i++)
	{
		memcpy(pTmp+i*(m_i4ColumnNumber+i4SecColNum), m_pData+i*m_i4ColumnNumber, i4ColumnNo*sizeof(f8));
		memcpy(pTmp+i*(m_i4ColumnNumber+i4SecColNum) + i4ColumnNo, pSecMatrix+i*i4SecColNum, i4SecColNum*sizeof(f8));
		if(i4ColumnNo != m_i4ColumnNumber)
			memcpy(pTmp+i*(m_i4ColumnNumber+i4SecColNum) + i4ColumnNo + i4SecColNum, 
				   m_pData+i*m_i4ColumnNumber+i4ColumnNo, (m_i4ColumnNumber-i4ColumnNo)*sizeof(f8));
	}
	delete []m_pData;
	m_pData = pTmp;
	m_i4ColumnNumber = m_i4ColumnNumber + i4SecColNum;
	return 1;
}

int4 CMatrix::InsertOneRowWithMember(int4 i4RowNo, f8 f8Element)
{
	if(i4RowNo > m_i4RowNumber) return 0;
	CMatrix SecMatrix;
	if(0 == SecMatrix.InitZeroMatrix(1, m_i4ColumnNumber)) return 0;
	f8* pData = SecMatrix.GetDataAddress();
	for(int4 i=0; i<m_i4ColumnNumber; i++) pData[i] = f8Element;
	return InsertMatrixInRow(i4RowNo,  SecMatrix);
}

int4 CMatrix::InsertOneColomnWithMember(int4 i4ColumnNo, f8 f8Element)
{
	if(i4ColumnNo > m_i4ColumnNumber) return 0;
	CMatrix SecMatrix;
	if(0 == SecMatrix.InitZeroMatrix(m_i4RowNumber, 1)) return 0;
	f8* pData = SecMatrix.GetDataAddress();
	for(int4 i=0; i<m_i4RowNumber; i++) pData[i] = f8Element;	
	return InsertMatrixInColomn(i4ColumnNo, SecMatrix);
}


int4 CMatrix::AppendNewRow(f8* pData, int4 i4DataNumber)
{
	if(NULL == pData || 0 >= i4DataNumber) return 0;
	if(i4DataNumber != m_i4ColumnNumber && m_i4ColumnNumber != 0) return 0;
	CMatrix mTmpMatri(pData, 1, i4DataNumber);
	if(0 == mTmpMatri.GetRowNumber()) return 0;
	return InsertMatrixInRow(m_i4RowNumber, mTmpMatri);
}

int4 CMatrix::AppendNewColumn(f8* pData, int4 i4DataNumber)
{
	if(NULL == pData || 0 >= i4DataNumber) return 0;
	if(i4DataNumber != m_i4RowNumber && m_i4ColumnNumber*m_i4RowNumber != 0) return 0;
	CMatrix mTmpMatri(pData, i4DataNumber, 1);
	if(0 == mTmpMatri.GetRowNumber()) return 0;
	return InsertMatrixInColomn(m_i4ColumnNumber, mTmpMatri);	 
}

int4 CMatrix::AppendMatrixInRow(const CMatrix& SecMatrix)
{
	return InsertMatrixInRow(m_i4RowNumber,SecMatrix);
}

int4 CMatrix::AppendMatrixInColumn(const CMatrix& SecMatrix)
{
	return InsertMatrixInColomn(m_i4ColumnNumber,SecMatrix);
}

int4 CMatrix::AppendMatrixInDiagonal(const CMatrix& SecMatrix) 
{
	int4 i4SecRow = SecMatrix.GetRowNumber();
    int4 i4SecColumn = SecMatrix.GetColumnNumber();
	if(i4SecColumn == 0) return 1;
	if(m_i4ColumnNumber == 0) 
		return InitMatrix(SecMatrix.GetDataAddress(), i4SecRow, i4SecColumn);
	CMatrix maRightMatrix, maZeroMatri1;
	if(0 == maZeroMatri1.InitZeroMatrix(i4SecRow, m_i4ColumnNumber)) return 0;
	if(0 == maRightMatrix.InitZeroMatrix(m_i4RowNumber, i4SecColumn)) return 0;
	if(0 == maRightMatrix.AppendMatrixInRow(SecMatrix)) return 0;
	
	if(0 == this->AppendMatrixInRow(maZeroMatri1)) return 0;
	if(0 == this->AppendMatrixInColumn(maRightMatrix)) return 0;
	
	return 1;
}

int4 CMatrix::GetOneRow(int4 i4RowNo, f8* pData) const
{
	if(i4RowNo >= m_i4RowNumber) return 0;
	memcpy(pData, m_pData + i4RowNo * m_i4ColumnNumber, sizeof(f8) * m_i4ColumnNumber );//拷贝数据

	return 1;
}

int4 CMatrix::GetOneRow(int4 i4RowNo, CMatrix& maRowMatrix) const
{
	if(i4RowNo >= m_i4RowNumber) return 0;
	if(0 == maRowMatrix.InitZeroMatrix(1, m_i4ColumnNumber)) return 0;
	return GetOneRow(i4RowNo, maRowMatrix.GetDataAddress());
}

CMatrix	CMatrix::GetOneRow(int4 i4RowNo)const
{
	CMatrix maRowMatrix;
	if(i4RowNo >= m_i4RowNumber) return maRowMatrix;
	maRowMatrix.InitZeroMatrix(1, m_i4ColumnNumber);
	GetOneRow(i4RowNo, maRowMatrix.GetDataAddress());
	return maRowMatrix;
}

int4 CMatrix::GetOneColumn(int4 i4ColumnNo, f8* pData)const
{
	if(i4ColumnNo >= m_i4ColumnNumber) return 0;
	CMatrix maTmp = GetTranspose();
	return maTmp.GetOneRow(i4ColumnNo, pData);
}

int4 CMatrix::GetOneColumn(int4 i4ColumnNo, CMatrix& maColumnMatrix)const
{
	if(i4ColumnNo >= m_i4ColumnNumber) return 0;
	if(0 == maColumnMatrix.InitZeroMatrix(m_i4RowNumber, 1)) return 0;
	return GetOneColumn(i4ColumnNo, maColumnMatrix.GetDataAddress());
}

CMatrix	CMatrix::GetOneColumn(int4 i4ColumnNo)const
{
	CMatrix maColumnMatrix;
	if(i4ColumnNo >= m_i4ColumnNumber) return maColumnMatrix;
	maColumnMatrix.InitZeroMatrix(m_i4RowNumber, 1);
	GetOneColumn(i4ColumnNo, maColumnMatrix.GetDataAddress());
	return maColumnMatrix;
}

CMatrix CMatrix::DeleteOneRow(int4 i4RowNo)
{
	CMatrix maRowMatrix;
	if(i4RowNo > m_i4RowNumber-1) return maRowMatrix;
	DeleteOneRow(i4RowNo, &maRowMatrix);

	return maRowMatrix;
}

int4 CMatrix::DeleteOneRow(int4 i4RowNo, CMatrix* pmaRowMatrix)
{
	if(i4RowNo > m_i4RowNumber-1) return 0;
	if(NULL != pmaRowMatrix) GetOneRow(i4RowNo, *pmaRowMatrix);
	if(i4RowNo != m_i4RowNumber-1)
	{
		memmove(m_pData+ i4RowNo * m_i4ColumnNumber, 
				m_pData+ (i4RowNo+1) * m_i4ColumnNumber, 
				(m_i4RowNumber - (i4RowNo +1)) * m_i4ColumnNumber * sizeof(f8));
 	}
	m_i4RowNumber--;
	return 1;
}

CMatrix CMatrix::DeleteOneColumn(int4 i4ColumnNo)
{
	CMatrix mTmpMatri; 
	if(i4ColumnNo > m_i4ColumnNumber-1) return mTmpMatri;
	DeleteOneColumn(i4ColumnNo, &mTmpMatri);
	return mTmpMatri;
}

int4 CMatrix::DeleteOneColumn(int4 i4ColumnNo, CMatrix* pmaColumnMatrix)
{
	if(i4ColumnNo > m_i4ColumnNumber-1) return 0;
	if(NULL != pmaColumnMatrix) GetOneColumn(i4ColumnNo, *pmaColumnMatrix);
	this->Transpose();
	this->DeleteOneRow(i4ColumnNo);
	this->Transpose();
	
	return 1;
}

int4 CMatrix::CombineRow(int4 i4RowNo, int4 i4ComMode)
{
	if(i4RowNo >= m_i4RowNumber) return 0;
	for(int4 i=0; i<m_i4RowNumber; i++)
	{
		if(i != i4RowNo)
		{
			for(int4 j=0; j<m_i4ColumnNumber; j++)
			{
				*(m_pData+ i * m_i4ColumnNumber + j ) += *(m_pData + i4RowNo * m_i4ColumnNumber + j ) * i4ComMode;//nRowNo从0起算
			}
		}
	}
	this->DeleteOneRow(i4RowNo);
	
	return 1;
}

int4 CMatrix::CombineColumn(int4 i4ColumnNo, int4 i4ComMode)
{
	if(i4ColumnNo >= m_i4ColumnNumber) return 0;
	for(int4 i=0; i<m_i4RowNumber; i++)
	{
		for(int4 j=0; j<m_i4ColumnNumber; j++)
		{
			if(j != i4ColumnNo)
			{
				*(m_pData+ i * m_i4ColumnNumber + j ) += *(m_pData + i * m_i4ColumnNumber + i4ColumnNo ) * i4ComMode;//nRowNo从0起算
			}
		}
	}
	this->DeleteOneColumn(i4ColumnNo);
	
	return 1;
}

f8 CMatrix::GetTrace()const
{
	f8 f8Trace = 0.0;
	if(m_i4RowNumber != m_i4ColumnNumber)
	{	
	f8Trace = 0.0;
	}
	else
	{
		for(int4 i=0;i<m_i4RowNumber;i++)
		{
			f8Trace += *(m_pData + i*(m_i4ColumnNumber+1));
		}
	}
	return f8Trace;	
}

f8 CMatrix::GetNorm()const
{
	int4 i4ElementNum = m_i4RowNumber * m_i4ColumnNumber;
	f8 f8Norm = 0.0;
	for(int4 i=0; i<i4ElementNum; i++)
	{
		f8Norm += m_pData[i] * m_pData[i];
	}
	
	return f8Norm;
}

int4 CMatrix::GetEigenvalues(f8* pEigenvalues)const
{
	pEigenvalues = NULL;
	return 1;
}

void CMatrix::Transpose(const f8* pm1, f8* pm2, int4 i4m, int4 i4n)const
{
	int4 i,j;
	for(i=0; i<i4m; i++)
		for(j=0; j<i4n; j++)
			pm2[j*i4m+i] = pm1[i*i4n+j];
	return;
}

void CMatrix::Mult(const f8* pm1, const f8* pm2, f8* pResult, int4 i_1, int4 j_12, int4 j_2)const
{
	int4 i,j,k;
	for(i=0; i<i_1; i++)
		for(j=0;j<j_2;j++)
		{
			pResult[i*j_2+j]= 0.0;
			for(k=0; k<j_12; k++)
				pResult[i*j_2+j] += pm1[i*j_12+k] * pm2[j+k*j_2];
		}
	return;
}

int4 CMatrix::InversMatrix(f8* pm1, int4 i4n)const
{ 
	int4 *pis,*pjs;
	int4 i,j,k,l,u,v;
	f8 temp,max_v;
	pis = new int4[i4n];
	pjs = new int4[i4n];
	if(pis==NULL || pjs==NULL)
	{
		delete []pis; 
		delete []pjs;
		return(0);
	}
	
	for(k=0; k<i4n; k++)
	{
		max_v = 0.0;
		for(i=k; i<i4n; i++)
			for(j=k; j<i4n; j++)
			{
				temp = fabs(pm1[i*i4n+j]);
				if( temp>max_v )
				{
			        max_v = temp; 
					pis[k] = i; 
					pjs[k] = j;
				}
			}
		if(max_v < EPSILON)
		{
			delete []pis; 
			delete []pjs;
			return(0);
		}
		if(pis[k]!=k)
			for(j=0; j<i4n; j++)
			{
			   u = k*i4n+j;
			   v = pis[k]*i4n+j;
			   temp = pm1[u]; 
			   pm1[u] = pm1[v];
			   pm1[v] = temp;
			}
		if(pjs[k] != k)
			for(i=0; i<i4n; i++)
			{
				u = i*i4n+k; v = i*i4n+pjs[k];
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
		l=k*i4n+k;
		pm1[l]=1.0/pm1[l];
		for(j=0; j<i4n; j++)
			if(j!=k)
			{
				u = k*i4n+j;
				pm1[u] *= pm1[l];
			}
		for(i=0; i<i4n; i++)
			if(i!=k)
				for(j=0; j<i4n; j++)
					if(j!=k)
					{
					  u = i*i4n+j;
					  pm1[u] -= pm1[i*i4n+k] * pm1[k*i4n+j];
					}
		for(i=0; i<i4n; i++)
			if(i != k)
			{
				u = i*i4n+k;
				pm1[u] *= -pm1[l];
			}
	}
	for(k=i4n-1; k>=0; k--)
	{
		if(pjs[k]!=k)
			for(j=0; j<i4n; j++)
			{
				u = k*i4n+j; v = pjs[k]*i4n+j;
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
		if(pis[k] != k)
			for(i=0; i<i4n; i++)
			{
				u=i*i4n+k; v=i*i4n+pis[k];
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
	}
	delete []pis; delete []pjs;

  return 1;

}

int4 CMatrix::SetZero()
{
	if(m_pData != NULL)
	{
		memset(m_pData, 0, sizeof(f8) * m_i4RowNumber * m_i4ColumnNumber);
	}
	return 1;
}

int4 CMatrix::SetMinus()
{
	int4 nElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	for(int4 i=0; i<nElementNumber; i++)
	{
		*(m_pData + i) *= -1;
	}
	
	return 1;
}

int4 CMatrix::Multiply(f8 f8MulNum)
{
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	for(int4 i=0;i<i4ElementNumber;i++)
	{
		m_pData[i] *= f8MulNum;
	}
	
	return 1;
}

int4 CMatrix::SetIdentity(int4 i4RowNumber)
{
	if(i4RowNumber <= 0) return 0;

	this->InitZeroMatrix(i4RowNumber, i4RowNumber);
	memset(m_pData, 0, sizeof(f8) * i4RowNumber * i4RowNumber);
	for(int4 j=0; j<i4RowNumber; j++)
	{
		*(m_pData + j*(i4RowNumber+1) ) = 1.0;
	}

	return 1;
}

int4 CMatrix::SetCorrelativity(int4 i4RowNumber)
{
	if(i4RowNumber==0) return 0;

	if(i4RowNumber * i4RowNumber != m_i4RowNumber * m_i4ColumnNumber)
	{
		this->InitZeroMatrix(i4RowNumber, i4RowNumber);
	}
	
	for(int4 i=0; i<i4RowNumber*i4RowNumber; i++)
	{
		*(m_pData + i ) = 1.0;
	}
	for(int4 j=0; j<i4RowNumber; j++)
	{
		*(m_pData + j*(i4RowNumber+1) ) = 2.0;
	}

	return 1;
}

///////////////////////////////////////////////////
//    operator override
///////////////////////////////////////////////////

//Func: get the element of matrix for set, example: A(2,3) = x;
//Para: i4RowNo		the row number, from 0
//		i4ColumnNo	the column number, from 0
//Retu: the element of A[i4RowNo,i4ColumnNo]
f8& CMatrix::operator()(int4 i4RowNo, int4 i4ColumnNo)
{
//	if(i4RowNo<0 || i4ColumnNo<0) return 0;
//	if(i4RowNo >= m_i4RowNumber || i4ColumnNo >= m_i4ColumnNumber) return 0;
	return *(m_pData + i4RowNo * m_i4ColumnNumber + i4ColumnNo);
}

//Func: get the element of matrix for get, example: x = A(2,3);
//Para: i4RowNo		the row number, from 0
//		i4ColumnNo	the column number, from 0
//Retu: the element of A[i4RowNo,i4ColumnNo]
f8 CMatrix::operator()(int4 i4RowNo, int4 i4ColumnNo)const
{
	return *(m_pData + i4RowNo * m_i4ColumnNumber + i4ColumnNo);
}


//CMatrix& CMatrix::operator= (const CMatrix & SecMatrix)
//{
//	if(&SecMatrix == this) return *this;
//	this->InitMatrix(SecMatrix.GetDataAddress(),SecMatrix.GetRowNumber(),SecMatrix.GetColumnNumber());
//
//	return *this;
//}

int4 CMatrix::operator= (const CMatrix& SecMatrix)
{
	if(&SecMatrix == this) return 1;
	return this->InitMatrix(SecMatrix.GetDataAddress(),SecMatrix.GetRowNumber(),SecMatrix.GetColumnNumber());
}

CMatrix CMatrix::operator+ (const CMatrix & SecMatrix)const
{
	CMatrix TmpMatrix;
	if((m_i4RowNumber != SecMatrix.GetRowNumber()) || (m_i4ColumnNumber != SecMatrix.GetColumnNumber())) return TmpMatrix;

	if(0 == TmpMatrix.InitZeroMatrix(m_i4RowNumber, m_i4ColumnNumber)) return TmpMatrix;
	f8* pTmpData = TmpMatrix.GetDataAddress();
	f8* pSecData = SecMatrix.GetDataAddress();
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		 *(pTmpData + i) = *(m_pData + i) + *(pSecData + i);
	}

	return TmpMatrix;
}

CMatrix CMatrix::operator- (const CMatrix & SecMatrix)const
{
	CMatrix TmpMatrix;
	if((m_i4RowNumber != SecMatrix.GetRowNumber()) || (m_i4ColumnNumber != SecMatrix.GetColumnNumber())) return TmpMatrix;

	if(0 == TmpMatrix.InitZeroMatrix(m_i4RowNumber, m_i4ColumnNumber)) return TmpMatrix;
	f8* pTmpData = TmpMatrix.GetDataAddress();
	f8* pSecData = SecMatrix.GetDataAddress();
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		 *(pTmpData + i) = *(m_pData + i) - *(pSecData + i);
	}

	return TmpMatrix;
}

CMatrix CMatrix::operator- ()
{
	CMatrix TmpMatrix(* this);
	TmpMatrix.SetMinus();
	return TmpMatrix;
}

CMatrix CMatrix::operator* (const CMatrix& SecMatrix)const
{
	CMatrix TmpMatrix;
	if(m_i4ColumnNumber != SecMatrix.m_i4RowNumber) return TmpMatrix;//A阵列数不等于B阵行数
	if(0 == TmpMatrix.InitZeroMatrix(m_i4RowNumber, SecMatrix.GetColumnNumber())) return TmpMatrix;
	int4 i4ElementNumber = m_i4RowNumber * SecMatrix.GetColumnNumber();
	f8* pTmpData = TmpMatrix.GetDataAddress();
	Mult(m_pData, SecMatrix.GetDataAddress(), pTmpData, m_i4RowNumber, m_i4ColumnNumber, SecMatrix.GetColumnNumber());

	return TmpMatrix;
}

CMatrix CMatrix::operator *(const f8 f8MultiNum) const
{
	CMatrix TmpMatrix(*this);
	if(0 == TmpMatrix.GetRowNumber()) return TmpMatrix;
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	f8* pTmpData = TmpMatrix.GetDataAddress();
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		pTmpData[i] *= f8MultiNum;
	}

	return TmpMatrix;
}

CMatrix CMatrix::operator /(const f8 f8DevideNum) const
{
	CMatrix TmpMatrix;
	if(f8DevideNum < EPSILON && f8DevideNum > -EPSILON) return TmpMatrix;
	TmpMatrix = this->operator * (1/f8DevideNum);

	return TmpMatrix;
}

CMatrix CMatrix::operator/ (const CMatrix & SecMatrix)const
{
	CMatrix TmpMatrix1; SecMatrix.GetInverse(TmpMatrix1);
	if(0 == TmpMatrix1.GetRowNumber()) return TmpMatrix1;//如果求逆失败，则返回空阵
	CMatrix TmpMatrix2 = (*this) * TmpMatrix1;
	return TmpMatrix2;
}

CMatrix CMatrix::operator% (const CMatrix & SecMatrix)const
{
	CMatrix TmpMatrix1; this->GetInverse(TmpMatrix1);
	if(0 == TmpMatrix1.GetRowNumber()) return TmpMatrix1;//如果求逆失败，则返回空阵
	CMatrix TmpMatrix2 = TmpMatrix1 * SecMatrix;

	return TmpMatrix2;
}

int4 CMatrix::operator+= (const CMatrix& SecMatrix)
{
	if((m_i4RowNumber != SecMatrix.GetRowNumber()) || (m_i4ColumnNumber != SecMatrix.GetColumnNumber())) return 0;
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	f8* pSecData = SecMatrix.GetDataAddress();
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		*(m_pData + i) += *(pSecData + i);
	}
	return 1;
}

int4 CMatrix::operator-= (const CMatrix& SecMatrix)
{
	if((m_i4RowNumber != SecMatrix.GetRowNumber()) || (m_i4ColumnNumber != SecMatrix.GetColumnNumber())) return 0;
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	f8* pSecData = SecMatrix.GetDataAddress();
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		*(m_pData + i) -= *(pSecData + i);
	}
	return 1;
}

bool CMatrix::operator== (const CMatrix& SecMatrix)const
{
	if((m_i4RowNumber != SecMatrix.GetRowNumber()) || (m_i4ColumnNumber != SecMatrix.GetColumnNumber())) return 0;
	int4 i4ElementNumber = m_i4RowNumber * m_i4ColumnNumber;
	f8* pSecData = SecMatrix.GetDataAddress();
	for(int4 i=0; i<i4ElementNumber; i++)
	{
		if(fabs(*(m_pData + i) - *(pSecData + i)) > EPSILON) return 0;
	}
	return 1;
}

bool CMatrix::operator!= (const CMatrix& SecMatrix)const
{
	if(*this == SecMatrix) return 0;
	else return 1;
}
