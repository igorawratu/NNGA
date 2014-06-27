#ifndef COMMON_H
#define COMMON_H

#include "pugixml.hpp"
#include <map>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

const double PI = 3.141592653589793238462;
const double e = 2.718281828459045;

#define BIT(x) (1<<(x))

typedef unsigned int uint;
typedef pugi::xml_document xmldoc;
typedef pugi::xml_node xmlnode;
typedef pugi::xml_attribute xmlattr;

template<class T> bool getParameter(map<string, T>& param, T& val, string key){
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
    vector3();
    vector3(double _x, double _y, double _z);

    double calcDistance(vector3 _to);

    vector3 normalize();

    vector3 operator * (const double& rhs);
    vector3 operator / (const double& rhs);
    vector3 operator + (const vector3& rhs);
    vector3 operator - (const vector3& rhs);

    double dotValue(const vector3& other);


    double x;
    double y;
    double z;
};

struct Line
{
    bool operator==(const Line& rhs);

    vector3 getMidpoint();

    vector3 p1;
    vector3 p2;
};

#endif