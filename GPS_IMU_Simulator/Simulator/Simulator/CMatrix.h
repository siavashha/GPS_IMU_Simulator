//////////////////////////////////////////////////////////////////////////////////////
//																					//
//			Matrix.h: interface for the matrix operation class.						//
//																					//
//       Copyright (c) 2008-2018, SPIN Lab. All rights reserved.					//
//																					//
//Purpose:																			//
//      matrix Initialize, the rwo/column/block operation, inverse/transpose/		//
//		orthogonalization, trace/eigenvalue/norn calculation, operator overload		//
//																					//
//Creation date: Nov, 2007															//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
typedef int int4;
typedef double f8;
//#undef NULL
//#define NULL = NULL
#define INFINITY (1e15)
#define EPSILON (1e-25)

#if !defined(__MATRIX_H__)
#define __MATRIX_H__

#ifdef PRE_EXPORTS
#define MATRIX_API __declspec(dllexport)
#else
#define MATRIX_API __declspec(dllimport)
#endif

class  CMatrix  
{
public:
	CMatrix();
	//copy construction
	CMatrix(const CMatrix& OriginalMatrix);
	//construct a matrix with elements valued zero
	CMatrix(int4 i4RowNumber, int4 i4ColumnNumber);
	//construct a matrix with input data
	CMatrix(const f8* pData, int4 i4RowNumber, int4 i4ColumnNumber);
	virtual ~CMatrix();

//functions
public:
	//Func: Initialize matrix
	//Para: pData:			the first address of raw input data
	//		i4RowNumber:	row number of matrix
	//		i4ColumnNumber: column number of matrix
	//Retu: success, 1; fail, 0
	int4	InitMatrix(const f8* pData,int4 i4RowNumber, int4 i4ColumnNumber);

	//Func: Initialize a matrix, and its elements are set as zero
	//Retu: success, 1; fail, 0
	int4	InitZeroMatrix(int4 i4RowNumber,int4 i4ColumnNumber);

	//Func: get the inverse matrix of this matrix, and this matrix is not changed
	//Para: maInverse: inverse matrix of this matrix
	//Retu: success, 1; fail, 0
	int4	GetInverse(CMatrix& maInverse)const;

	//Func: get the inverse matrix of this matrix, and this matrix is not changed
	//Retu: success, inverse matrix of this matrix; fail, an empty matrix
	CMatrix GetInverse()const;

	//Func: inverse this matrix
	//Retu: success, 1; fail, 0
	int4	Inverse();

	//Func: get the transpose of this matrix, and this matrix is not changed
	//Para: maTransposed: transpose of this matrix
	//Retu: success, 1; fail, 0
	int4	GetTranspose(CMatrix& maTransposed)const;

	//Func: get the transpose of this matrix, and this matrix is not changed
	//Retu: success, transpose of this matrix; fail, an empty matrix
	CMatrix GetTranspose()const;

	//Func: transpose this matrix
	//Retu: success, 1; fail, 0
	int4	Transpose();
	
	//Func: get a matrix with the elements that are the rounded values of the elements of this matrix,
	//  	and this matrix is not changed
	//Para: maRounded: the matrix that is rounded of this matrix
	//Retu: success, 1; fail, 0
	//int4	GetRoundMatrix(CMatrix& maRounded)const;

	//Func: get a matrix with the elements that are the rounded values of the elements of this matrix,
	//  	and this matrix is not changed
	//Retu: success, the matrix that is rounded of this matrix; fail, an empty matrix
	//CMatrix GetRoundMatrix()const;
	
	//Func: round the elements of this matrix
	//Retu: success, 1; fail, 0
	//int4	Round();
	
	//Func: Orthogonalize the matrix
	//Retu: success, 1; fail, 0
	int4	Orthogonalize();

	//Func: get the up left corner submatrix of this matrix, and this matrix is not changed
	//Para: i4SubRowNumber:		the row number of the submatrix
	//		i4SubColumnNumber:	the column number of the submatrix
	//		maSubMatrix:		the submatrix
	//Retu: success, 1; fail, 0
	int4	GetLeftTopSubMatrix(int4 i4SubRowNumber, int4 i4SubColumnNumber, CMatrix& maSubMatrix)const;

	//Func: get the up left corner submatrix of this matrix, and this matrix is not changed
	//Para: i4SubRowNumber:		the row number of the submatrix
	//		i4SubColumnNumber:	the column number of the submatrix
	//Retu: success, the submatrix; fail, an empty matrix
	CMatrix GetLeftTopSubMatrix(int4 i4SubRowNumber,int4 i4SubColumnNumber)const;

	//Func: get the submatrix in arbitrary location of this matrix, and this matrix is not changed
	//Para: nBeginRowNo: the beginning row serial number of the submatrix in this matrix
	//		nSubRowNumber:		the row number of the submatrix
	//		nBeginColumnNo:		the beginning column serial number of the submatrix in this matrix
	//		nSubColumnNumber:	the column number of the submatrix
	//		maSubMatrix: the submatrix
	//Retu: success, 1; fail, 0
	int4	GetSubMatrix(int4 i4BeginRowNo, int4 i4SubRowNumber, int4 i4BeginColumnNo, int4 i4SubColumnNumber, CMatrix& maSubMatrix)const;

	//Func: get the submatrix in arbitrary location of this matrix, and this matrix is not changed
	//Para: nBeginRowNo:	the beginning row serial number of the submatrix in this matrix
	//		nSubRowNumber:	the row number of the submatrix
	//		nBeginColumnNo:	the beginning column serial number of the submatrix in this matrix
	//		nSubColumnNumber:the column number of the submatrix
	//		maSubMatrix:	the submatrix
	//Retu: success, the submatrix; fail, an empty matrix
	CMatrix GetSubMatrix(int4 i4BeginRowNo, int4 i4SubRowNumber, int4 i4BeginColumnNo, int4 i4SubColumnNumber)const;
	
	//Func: cerate a new matrix by adding this matrix with other matrix, and this matrix is not changed
	//Para: maRight:	the matrix is used to add
	//		maResult:	the result matrix of the two matrices added
	//Retu: success, 1; fail, 0
	//Exam: A.AddMatrix(B, C): C = A + B
	int4	AddMatrix(CMatrix& maRight, CMatrix& maResult)const;

	//Func: create a new matrix by subtracting a matrix from this matrix, and this matrix is not changed
	//Para: maRight:	the matrix is used to add
	//		maResult:	the result matrix
	//Retu: success, 1; fail, 0
	//Exam: A.SubMatrix(B, C): C = A - B
	int4	SubMatrix(CMatrix& maRight, CMatrix& maResult)const;
	
	//Func: create a new matrix by multiplying a matrix from this matrix, and this matrix is not changed
	//Para: maRight:	the matrix is used to multiply
	//		maResult:	the result matrix
	//Retu: success, 1; fail, 0
	//Exam: A.MultiplyMatrix(B, C): C = A * B
	int4	MultiplyMatrix(CMatrix& maRight, CMatrix& maResult)const;
	
	//Func: change the submatrix in this matrix by new one
	//Para: i4BeginRowNo:	the row serial number of the submatrix in this matrix, it begins with 0
	//		i4BeginColumnNo:the column serial number of the submatrix in this matrix, it begins with 0
	//		maSecMatrix:	the new submatrix to replace the original one
	//Retu: success, 1; fail, 0
	int4    ChangeSubMatrix(int4 i4BeginRowNo, int4 i4BeginColumnNo, const CMatrix& maSecMatrix);
	
	//Func: add the submatrix in this matrix by another one
	//Para: i4BeginRowNo:	the row serial number of the submatrix in this matrix, it begins with 0
	//		i4BeginColumnNo:the column serial number of the submatrix in this matrix, it begins with 0
	//		maSecMatrix:	the new submatrix to add to the original one
	//Retu: success, 1; fail, 0
	int4    AddSubMatrixMember(int4 i4BeginRowNo, int4 i4BeginColumnNo, const CMatrix& maSecMatrix);

	//Func: change onw row of this matrix by one value
	//Para: i4RowNo:	the row serial number of the row to be changed, it begins with 0
	//		f8Element:	the value used to change the row
	//Retu: success, 1; fail, 0
	int4	ChangeOneRowWithMember(int4 i4RowNo, f8 f8Element);
	
	//Func: change one row of this matrix by one column
	//Para: i4ColumnNo:	the column serial number of the column to be changed, it begins with 0
	//		f8Element:	the value used to change the column
	//Retu: success, 1; fail, 0
	int4	ChangeOneColumnWithMember(int4 i4ColumnNo, f8 f8Element);
	
	//Func: get one element of the matrix
	//Para: i4RowNo:	the row serial number of the element, it begins with 0
	//		i4ColumnNo:	the column serial number of the element, it begins with 0
	//		f8Element:	the element value
	//Retu: success, 1; fail, 0
	int4	GetElement(int4 i4RowNo, int4 i4ColumnNo, f8& f8Element)const;

	//Func: get one element of the matrix
	//Para: i4RowNo:	the row serial number of the element, it begins with 0
	//		i4ColumnNo:	the column serial number of the element, it begins with 0
	//Retu:	the element value
	f8		GetElement(int4 i4RowNo, int4 i4ColumnNo)const;
	
	//Func: change one element of the matrix
	//Para: i4RowNo:	the row serial number of the element, it begins with 0
	//		i4ColumnNo:	the column serial number of the element, it begins with 0
	//		f8Element:	the element value used to change original one
	//Retu: success, 1; fail, 0
	int4	ChangeElement(int4 i4RowNo, int4 i4ColumnNo, f8 f8Element)const;

	//Func: exchange two rows of the matrix
	//Para: i4RowNo1, i4RowNo2:	the row serial numbers of two rows, it begins with 0
	//Retu: success, 1; fail, 0
	int4	ExChangeTwoRows(int4 i4RowNo1, int4 i4RowNo2)const;

	//Func: exchange two columns of the matrix
	//Para: i4ColumnNo1, i4ColumnNo2: the column serial numbers of two columns, it begins with 0
	//Retu: success, 1; fail, 0
	int4	ExChangeTwoColumns(int4 i4ColumnNo1, int4 i4ColumnNo2)const;

	//Func: insert new row into the matrix, and it's tenable for the empty matrix
	//Para: i4RowNo:	the row serial number on that the new row will be inserted, it begins with 0
	//		pData:		the first address of the row used to insert
	//		nDataNumber:the data length, that is the column number of the original matrix
	//Retu: success, 1; fail, 0
	int4	InsertNewRow(int4 i4RowNo, f8* pData, int4 i4DataNumber);
	
	//Func: insert new column into the matrix, and it's tenable for the empty matrix
	//Para: i4ColumnNo:	the column serial number on that the new column will be inserted, it begins with 0
	//		pData:		the first address of the column used to insert
	//		nDataNumber:the data length, that is the row number of the original matrix
	//Retu: success, 1; fail, 0
	int4	InsertNewColumn(int4 i4ColumnNo, f8* pData, int4 i4DataNumber);

	//Func: insert a row matrix into the matrix, and it's tenable for the empty matrix
	//Para: i4RowNo:	the row serial number on that the row matrix will be inserted, it begins with 0
	//		SecMatrix:	the matrix will be inserted
	//Retu: success, 1; fail, 0
	int4	InsertMatrixInRow(int4 i4RowNo, const CMatrix& SecMatrix); 

	//Func: insert a column matrix into the matrix, and it's tenable for the empty matrix
	//Para: i4ColumnNo:	the column serial number on that the column matrix will be inserted, it begins with 0
	//		SecMatrix:	the matrix will be inserted
	//Retu: success, 1; fail, 0
	int4	InsertMatrixInColomn(int4 i4ColumnNo, const CMatrix& SecMatrix); 

	//Func: insert new row into the matrix with same elements, and it's tenable for the empty matrix
	//Para: i4RowNo:	the row serial number on that the new row will be inserted, it begins with 0
	//		f8Element:	the value of the elements to insert
	//Retu: success, 1; fail, 0
	int4	InsertOneRowWithMember(int4 i4RowNo, f8 f8Element); 
	
	//Func: insert new column into the matrix with same elements, and it's tenable for the empty matrix
	//Para: i4ColumnNo:	the column serial number on that the new row will be inserted, it begins with 0
	//		f8Element:	the value of the elements to insert
	//Retu: success, 1; fail, 0
	int4	InsertOneColomnWithMember(int4 i4ColumnNo, f8 f8Element); 

	//Func: append new row to the matrix, and it's tenable for the empty matrix
	//Para: pData:		the first address of the row used to insert
	//		nDataNumber:the data length, that is the column number of the original matrix
	//Retu: success, 1; fail, 0
	int4	AppendNewRow(f8* pData, int4 i4DataNumber);

	//Func: append new column to the matrix, and it's tenable for the empty matrix
	//Para: pData:		the first address of the column used to insert
	//		nDataNumber:the data length, that is the row number of the original matrix
	//Retu: success, 1; fail, 0
	int4	AppendNewColumn(f8* pData, int4 i4DataNumber);

	//Func: append a new matrix to the matrix in row, and it's tenable for the empty matrix
	//Para: SecMatrix:	the matrix to append
	//Retu: success, 1; fail, 0
	int4	AppendMatrixInRow(const CMatrix& SecMatrix);

	//Func: append a new matrix to the matrix in column, and it's tenable for the empty matrix
	//Para: SecMatrix:	the matrix to append
	//Retu: success, 1; fail, 0
	int4	AppendMatrixInColumn(const CMatrix& SecMatrix);
	
	//Func: append a new matrix to the matrix in diagonal, the off-diagonal is set as zero,
	//		it's tenable for the empty original matrix. If the matrix used to append is empty, the original matrix dosen't change
	//Para: SecMatrix:	the matrix to append
	//Retu: success, 1; fail, 0
	//Exam: 
	//	A.AppendMatrixInDiagonal( B ) ->| A 0|
	//									| 0 B|
	int4	AppendMatrixInDiagonal(const CMatrix& SecMatrix); 
	
	//Func: get one row of the matrix
	//Para: i4RowNo:	the row serial number of the data, it begins with 0
	//		pData:		the first address of the buffer to save data
	//Retu: success, 1; fail, 0
	//Note: pData should be a valid address, and the space size should be equal to or more than the column number of the matrix
	int4	GetOneRow(int4 i4RowNo, f8* pData)const;

	//Func: get one row submatrix of the matrix
	//Para: i4RowNo:	the row serial number of the data, it begins with 0
	//		maRowMatrix:the row submatrix, its space is created in the func
	//Retu: success, 1; fail, 0
	int4	GetOneRow(int4 i4RowNo, CMatrix& maRowMatrix)const;

	//Func: get one row submatrix of the matrix
	//Para: i4RowNo:	the row serial number of the data, it begins with 0
	//Retu: the row submatrix, its space is created in the func
	CMatrix	GetOneRow(int4 i4RowNo)const;
	
	//Func: get one column of the matrix
	//Para: i4ColumnNo:	the column serial number of the data, it begins with 0
	//		pData:		the first address of the buffer to save data
	//Retu: success, 1; fail, 0
	//Note: pData should be a valid address, and the space size should be equal to or more than the row number of the matrix
	int4	GetOneColumn(int4 i4ColumnNo, f8* pData)const;

	//Func: get one column submatrix of the matrix
	//Para: i4ColumnNo:	the column serial number of the data, it begins with 0
	//		maColumnMatrix:the column submatrix, its space is created in the func
	//Retu: success, 1; fail, 0
	int4	GetOneColumn(int4 i4ColumnNo, CMatrix& maColumnMatrix)const;

	//Func: get one column submatrix of the matrix
	//Para: i4ColumnNo:	the column serial number of the data, it begins with 0
	//Retu: maColumnMatrix:the column submatrix, its space is created in the func
	CMatrix	GetOneColumn(int4 i4ColumnNo)const;
	
	//Func: delete one row of the matrix
	//Para: i4RowNo:	the row serial number to delete, it begins with 0
	//Retu: the row submatrix deleted
	CMatrix	DeleteOneRow(int4 i4RowNo);

	//Func: delete one row of the matrix
	//Para: i4RowNo:	the row serial number to delete, it begins with 0
	//		pmaRowMatrix: the address of the row submatrix deleted
	//Retu: success, 1; fail, 0
	int4	DeleteOneRow(int4 i4RowNo, CMatrix* pmaRowMatrix);
	
	//Func: delete one column of the matrix
	//Para: i4ColumnNo:	the column serial number to delete, it begins with 0
	//Retu: the column submatrix deleted
	CMatrix	DeleteOneColumn(int4 i4ColumnNo);

	//Func: delete one column of the matrix
	//Para: i4ColumnNo:	the column serial number to delete, it begins with 0
	//		pmaColumnMatrix: the address of the column submatrix deleted
	//Retu: success, 1; fail, 0
	int4	DeleteOneColumn(int4 i4ColumnNo, CMatrix* pmaColumnMatrix);
	
	//Func: combine other rows of the matrix with certain row, and then this row is deleted
	//Para: i4RowNo:	the row serial number of the row to be combined, it begins with 0
	//		i4ComMode:	the combination mode: 1, adding; -1, subtraction
	//Retu: success, 1; fail, 0
	int4	CombineRow(int4 i4RowNo, int4 i4ComMode);

	//Func: combine other columns of the matrix with certain column, and then this column is deleted
	//Para: i4ColumnNo:	the column serial number of the column to be combined, it begins with 0
	//		i4ComMode:	the combination mode: 1, adding; -1, subtraction
	//Retu: success, 1; fail, 0
	int4	CombineColumn(int4 i4ColumnNo, int4 i4ComMode);
	
	//Func: dump the matrix, i.e. the row and column of the matrix are zore after the operation
	//Retu: success, 1; fail, 0
	int4	Dump();

	//Func: set the matrix elements as zero
	//Retu: 1
	int4	SetZero();

	//Func: set the matrix elements as minus original elements
	//Retu: 1
	int4	SetMinus();

	//Func: multiply the matrix with a real scalar
	//Para: f8MulNum:	the number to multiply
	//Retu: 1
	int4	Multiply(f8 f8MulNum);

	//Func: set or reset the matrix as a identity
	//Para: i4RowNumber: the dimension of the matrix, it should be a positive integer
	//Retu: success, 1; fail, 0
	int4	SetIdentity(int4 i4RowNumber);

	//Func: set or reset the matrix as a simple correlative matrix
	//Para: i4RowNumber: the dimension of the matrix, it should be a positive integer
	//Retu: success, 1; fail, 0
	//Exam: A.SetCorrelativity(3)	->	|2 1 1 |
	//									|1 2 1 |
	//									|1 1 2 |
	int4	SetCorrelativity(int4 i4RowNumber);


//operator overload
public:
	f8& operator()(int4 i4RowNo, int4 i4ColumnNo);//A(2,3) = x;
	f8 operator()(int4 i4RowNo, int4 i4ColumnNo)const;//x = A(2,3);
	int4 operator= (const CMatrix& SecMatrix);//A = B
	CMatrix	operator+ (const CMatrix& SecMatrix)const;//C = A + B
	CMatrix operator- (const CMatrix& SecMatrix)const;//C = A - B
	CMatrix operator- ();//C = - A
	CMatrix operator* (const CMatrix& SecMatrix)const;//C = A * B
	CMatrix operator* (const f8 f8MultiNum) const;//C = A * b
	CMatrix operator/ (const f8 f8DevideNum) const;//C = A / b
	CMatrix operator/ (const CMatrix& SecMatrix)const;//C = A / B	-> C = A * B.Inverse()
	CMatrix operator% (const CMatrix& SecMatrix)const;//C = A % B	-> C = A.Inverse() * B
	
	int4 operator+= (const CMatrix& SecMatrix);//A += B		-> A = A + B
	int4 operator-= (const CMatrix& SecMatrix);//A -= B		-> A = A - B
	bool operator== (const CMatrix& SecMatrix)const;
	bool operator!= (const CMatrix& SecMatrix)const;

//attribute interface
public:
	//Func: get the data of the matrix
	//Para: pDestiData: the buffer to save the data
	//Retu: 1
	int4	GetData(f8 * pDestiData)const;

	//Func: get the data address of the buffer in the matrix saving the data
	//Retu: the buffer data address
	//Note: this interface destroys the enclosure of the class, so any changing operation of the address is forbidden
	f8*     GetDataAddress()const;

	//Func: get the row number of the matrix
	int4	GetRowNumber()const {return m_i4RowNumber;};
	//Func: get the column number of the matrix
	int4	GetColumnNumber()const {return m_i4ColumnNumber;};

	//Func: get the trace of the square matrix
	f8	    GetTrace()const;
	//Func: get the norm of the matrix
	f8	    GetNorm()const;
	//Func: get the eigenvalues of the matrix
	int4	GetEigenvalues(f8* pEigenvalues)const;
	

//protected operation
private:
	void	Initate();
	void	Transpose(const f8* pm1, f8* pm2, int4 i4m, int4 i4n)const;
	void	Mult(const f8* pm1, const f8* pm2, f8* pResult, int4 i_1, int4 j_12, int4 j_2)const;
	int4	InversMatrix(f8* pm1, int4 i4n)const;

//attribute
protected:
	f8*   m_pData;//the buffer save data
	int4  m_i4RowNumber, m_i4ColumnNumber;//the numbers of row and column of the matrix
	
private://forbidden operation
};


#endif // !defined(__MATRIX_H__)
