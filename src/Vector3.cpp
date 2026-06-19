#include <stdexcept>
#include <cmath>
#include "constants.h"
#include "Vector3.h"
using namespace std;

// Constructors

Vector3::Vector3(){
    x=0.0;
    y=0.0;
    z=0.0;
}

Vector3::Vector3(double x,double y, double z){
    this->x=x;
    this->y=y;
    this->z=z;
}

Vector3::Vector3(double m[3]){
    x=m[0];
    y=m[1];
    z=m[2];
}

// Operators

Vector3 Vector3::operator+(const Vector3& other) const{
    return Vector3(x+other[0],y+other[1],z+other[2]);
}

Vector3 Vector3::operator-(const Vector3& other) const{
    return Vector3(x-other[0],y-other[1],z-other[2]);
}

Vector3 Vector3::operator*(double scalar) const{
    return Vector3(x*scalar,y*scalar,z*scalar);
}

Vector3 Vector3::operator/(double scalar) const{
    return Vector3(x/scalar,y/scalar,z/scalar);
}

double Vector3::operator*(const Vector3& other) const{
    return (x*other[0] + y*other[1] + z*other[2]);
}

Vector3 Vector3::operator^(const Vector3& other) const{
    return Vector3(y*other[2]-z*other[1],z*other[0]-x*other[2],x*other[1]-y*other[0]);
}

// To access the elements

double& Vector3::operator[](int t){
    if(t==0)
        return x;
    else if (t==1)
        return y;
    else if (t==2)
        return z;
    else
        throw out_of_range("Index out of range");
}

const double& Vector3::operator[](int t) const{
    if(t==0)
        return x;
    else if (t==1)
        return y;
    else if (t==2)
        return z;
    else
        throw out_of_range("Index out of range");
}

double Vector3::magnitude() const{
    return sqrt(x*x+y*y+z*z);
}

Vector3 Vector3::normalise() const{
    return (*this)/max(this->magnitude(),denMin);
}

Vector3 Vector3::hadamard(const Vector3& other) const{
    return Vector3(x*other[0],y*other[1],z*other[2]);
}

// To check if the vector is null vector

bool Vector3::isZero() const{
    if(this->magnitude()==0.0)
        return true;
    else
        return false;
}