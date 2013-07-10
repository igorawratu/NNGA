#ifndef GAENGINE_H
#define GAENGINE_H

#include <string>

#include "common.h"
#include "solution.h"

using namespace std;

class GAEngine
{
public:
    GAEngine();
    ~GAEngine();

    Solution train(string _geneticAlgorithm, string _simulation);

private:
    GAEngine(const GAEngine& other){}
    GAEngine& operator = (const GAEngine& other){}
    

};

#endif