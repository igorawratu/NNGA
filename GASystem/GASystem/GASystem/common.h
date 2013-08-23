#ifndef COMMON_H
#define COMMON_H

#include "pugixml.hpp"
#include <map>
#include <string>
#include <iostream>
#include <cmath>

using namespace std;

typedef unsigned int uint;
typedef pugi::xml_document xmldoc;
typedef pugi::xml_node xmlnode;
typedef pugi::xml_attribute xmlattr;

bool inline getParameter(map<string, double>& param, double& val, string key){
    map<string, double>::const_iterator iter = param.find(key);
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
};

float inline calcEucDistance(vector3 from, vector3 to){
    return sqrtf((to.x - from.x)*(to.x - from.x) + (to.y - from.y)*(to.y - from.y) + (to.z - from.z)*(to.z - from.z));
}

#endif