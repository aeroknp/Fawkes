#include <stdexcept>
#include <cmath>
#include "constants.h"
#include "Matrix.h"
using namespace std;

// Constructors

Matrix::Matrix(){
    m[0][0]=1.0,m[0][1]=0.0,m[0][2]=0.0;
    m[1][0]=0.0,m[1][1]=1.0,m[1][2]=0.0;                        // Initialising to identity matrix
    m[2][0]=0.0,m[2][1]=0.0,m[2][2]=1.0;
}

Matrix::Matrix(double a00,double a01,double a02,
               double a10,double a11,double a12,
               double a20,double a21,double a22)
{
    m[0][0]=a00,m[0][1]=a01,m[0][2]=a02;
    m[1][0]=a10,m[1][1]=a11,m[1][2]=a12;                        // Initialising to given matrix
    m[2][0]=a20,m[2][1]=a21,m[2][2]=a22;
}

// Operators

Matrix Matrix::operator+(const Matrix& other) const{
    return Matrix(m[0][0]+other(0,0),m[0][1]+other(0,1),m[0][2]+other(0,2),
                  m[1][0]+other(1,0),m[1][1]+other(1,1),m[1][2]+other(1,2),         // Matrix + Matrix
                  m[2][0]+other(2,0),m[2][1]+other(2,1),m[2][2]+other(2,2));
}

Matrix Matrix::operator-(const Matrix& other) const{
    return Matrix(m[0][0]-other(0,0),m[0][1]-other(0,1),m[0][2]-other(0,2),
                  m[1][0]-other(1,0),m[1][1]-other(1,1),m[1][2]-other(1,2),         // Matrix - Matrix
                  m[2][0]-other(2,0),m[2][1]-other(2,1),m[2][2]-other(2,2));
}

Matrix Matrix::operator*(double scalar) const{
    return Matrix(m[0][0]*scalar,m[0][1]*scalar,m[0][2]*scalar,
                  m[1][0]*scalar,m[1][1]*scalar,m[1][2]*scalar,                     // Matrix * double
                  m[2][0]*scalar,m[2][1]*scalar,m[2][2]*scalar);
}

Matrix operator*(double scalar,const Matrix& m){
    return Matrix(m(0,0)*scalar,m(0,1)*scalar,m(0,2)*scalar,
                  m(1,0)*scalar,m(1,1)*scalar,m(1,2)*scalar,                        // double * Matrix
                  m(2,0)*scalar,m(2,1)*scalar,m(2,2)*scalar);
}


Matrix Matrix::operator/(double scalar) const{
    if(abs(scalar)<denMin)
        if(scalar<0)
            scalar=-denMin;
        else
            scalar=denMin;

    return Matrix(m[0][0]/scalar,m[0][1]/scalar,m[0][2]/scalar,
                  m[1][0]/scalar,m[1][1]/scalar,m[1][2]/scalar,                     // Matrix / double
                  m[2][0]/scalar,m[2][1]/scalar,m[2][2]/scalar);
}

Vector3 Matrix::operator*(const Vector3& other) const{
    Vector3 row0(m[0][0],m[0][1],m[0][2]);
    Vector3 row1(m[1][0],m[1][1],m[1][2]);                                          // Matrix * Vector3
    Vector3 row2(m[2][0],m[2][1],m[2][2]);

    return Vector3(row0*other,row1*other,row2*other);
}

Matrix Matrix::operator*(const Matrix& other) const{
    
    Matrix result;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            result(i,j)=0.0;                                                       // Matrix * Matrix
            for(int k=0;k<3;k++){
                result(i,j)+=m[i][k]*other(k,j);
            }
        }
    }

    return result;
}

// To get certain values of matrix

double& Matrix::operator()(int row,int col){
    if(row>2 || col>2 || row<0 || col<0)
        throw out_of_range("The indices are out of range");
    else
        return m[row][col];                                                         // Get the element at the location
}

const double& Matrix::operator()(int row,int col) const{
    if(row>2 || col>2 || row<0 || col<0)
        throw out_of_range("The indices are out of range");
    else
        return m[row][col];
}

Matrix Matrix::transpose() const{

    Matrix result;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){                                                       // Get the transpose of the matrix
            result(i,j)=m[j][i];
        }
    }

    return result;
}

double Matrix::determinant() const{
    return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-
            m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+                              // Get the determinant of the matrix
            m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]));
}

