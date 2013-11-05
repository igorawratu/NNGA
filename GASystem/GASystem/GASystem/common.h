#ifndef COMMON_H
#define COMMON_H

#include "pugixml.hpp"
#include <map>
#include <string>
#include <iostream>
#include <cmath>

const double PI = 3.141592653589793238462;

using namespace std;

typedef unsigned int uint;
typedef pugi::xml_document xmldoc;
typedef pugi::xml_node xmlnode;
typedef pugi::xml_attribute xmlattr;

template<class T> bool inline getParameter(map<string, T>& param, T& val, string key){
    map<string, T>::const_iterator iter = param.find(key);
    if(iter == param.end()){
        cerr << "Error: unable to find " << key << " parameter for algorithm" << endl;
        return false;        
    }
    val = iter->second;
    return true;
}

struct vector3
{
    vector3() : x(0), y(0), z(0){}
    vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z){}
    double x;
    double y;
    double z;

    double calcDistance(vector3 _to){
        double finalx = _to.x - x, finaly = _to.y - y, finalz = _to.z - z;

        return sqrt(finalx*finalx + finaly*finaly + finalz*finalz);
    }

    void normalize(){
        double length = calcDistance(vector3(0, 0, 0));
        x /= length; y /= length; z /= length;
    }

    vector3 operator * (const double& rhs){
        x *= rhs;
        y *= rhs;
        z *= rhs;

        return *this;
    }

    vector3 operator / (const double& rhs){
        x /= rhs;
        y /= rhs;
        z /= rhs;

        return *this;
    }

    vector3 operator + (const vector3& rhs){
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;

        return *this;
    }

    vector3 operator - (const vector3& rhs){
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;

        return *this;
    }

    double dotValue(const vector3& other){
        return x * other.x + y * other.y + z * other.z;
    }


};

float inline calcEucDistance(vector3 from, vector3 to){
    return sqrtf((to.x - from.x)*(to.x - from.x) + (to.y - from.y)*(to.y - from.y) + (to.z - from.z)*(to.z - from.z));
}

struct Line
{
    bool operator==(const Line& rhs){ 
        bool one = (p1.x == rhs.p1.x && p1.y == rhs.p1.y && p1.z == rhs.p1.z) && (p2.x == rhs.p2.x && p2.y == rhs.p2.y && p2.z == rhs.p2.z);
        bool two = (p1.x == rhs.p2.x && p1.y == rhs.p2.y && p1.z == rhs.p2.z) && (p2.x == rhs.p1.x && p2.y == rhs.p1.y && p2.z == rhs.p1.z);

        return one || two;
    }

    vector3 p1;
    vector3 p2;
};

#endif