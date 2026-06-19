// This contains a custom matrix class for the simulation
#pragma once
#include "Vector3.h"

class Matrix{
    // Data of the matrix
    double m[3][3];

    public:
        // Constructors
        Matrix();
        Matrix(double a00,double a01,double a02,
               double a10,double a11,double a12,
               double a20,double a21,double a22);

        // Operators
        Matrix operator+(const Matrix& other) const;
        Matrix operator-(const Matrix& other) const;
        Matrix operator*(double scalar) const;
        friend Matrix operator*(double scalar,const Matrix& other);
        Matrix operator/(double scalar) const;
        Vector3 operator*(const Vector3& other) const;
        Matrix operator*(const Matrix& other) const;

        // To get certain values of matrix
        double& operator()(int row, int col);
        const double& operator()(int row, int col) const;
        Matrix transpose() const;
        double determinant() const;
        
};