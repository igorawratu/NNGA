#ifndef COMMON_H
#define COMMON_H

#include "pugixml.hpp"
#include <map>
#include <string>
#include <iostream>

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
    double x;
    double y;
    double z;
};

#endif