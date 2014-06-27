#include "common.h"

vector3::vector3() : x(0), y(0), z(0){}

vector3::vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z){}

double vector3::calcDistance(vector3 _to){
    double finalx = _to.x - x, finaly = _to.y - y, finalz = _to.z - z;

    return sqrt(finalx*finalx + finaly*finaly + finalz*finalz);
}

vector3 vector3::normalize(){
    vector3 newVec;
    double length = calcDistance(vector3(0, 0, 0));
    newVec.x = x / length; 
    newVec.y = y / length; 
    newVec.z = z / length;

    return newVec;
}

vector3 vector3::operator * (const double& rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;

    return *this;
}

vector3 vector3::operator / (const double& rhs){
    x /= rhs;
    y /= rhs;
    z /= rhs;

    return *this;
}

vector3 vector3::operator + (const vector3& rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

vector3 vector3::operator - (const vector3& rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;

    return *this;
}

double vector3::dotValue(const vector3& other){
    return x * other.x + y * other.y + z * other.z;
}


bool Line::operator==(const Line& rhs){ 
    bool one = (p1.x == rhs.p1.x && p1.y == rhs.p1.y && p1.z == rhs.p1.z) && (p2.x == rhs.p2.x && p2.y == rhs.p2.y && p2.z == rhs.p2.z);
    bool two = (p1.x == rhs.p2.x && p1.y == rhs.p2.y && p1.z == rhs.p2.z) && (p2.x == rhs.p1.x && p2.y == rhs.p1.y && p2.z == rhs.p1.z);

    return one || two;
}

vector3 Line::getMidpoint(){
    return vector3((p1.x + p2.x)/2, (p1.y + p2.y)/2, (p1.z + p2.z)/2);
}