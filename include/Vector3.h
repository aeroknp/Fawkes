// This file contains a custom vector class used during this simulation
#pragma once

class Vector3{
    // The vector variables
        double x;
        double y;
        double z;
        
    public:
        // Constructors 
        Vector3();
        Vector3(double x,double y, double z);
        Vector3(double m[3]);

        // Operators
        Vector3 operator+(const Vector3& other) const;          // Vector Addition 
        Vector3 operator-(const Vector3& other) const;          // Vector Subtraction
        Vector3 operator*(double scalar) const;                 // Scalar Multiplication
        Vector3 operator/(double scalar) const;                 // Scalar Divison
        double operator*(const Vector3& other) const;           // Dot product
        Vector3 operator^(const Vector3& other) const;          // Cross product

        // To get some values of a vector
        double& operator[](int t);                        // Return an element of the vector
        const double& operator[](int t) const;            // Return a non modifiable element of the vector
        double magnitude() const;                         // Get the magnitude of the vector
        Vector3 normalise() const;                        // Get the normalised vector
        Vector3 hadamard(const Vector3&) const;           // Get element wise multiplication of two vectors 
       
        // To check if the vector is null vector
        bool isZero() const;                              
};