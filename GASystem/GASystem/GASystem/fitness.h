#ifndef FITNESS
#define FITNESS

#include <vector>
#include <map>

using namespace std;

class Fitness
{
public:
    Fitness(){}
    virtual ~Fitness(){}

    virtual double evaluateFitness(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc)=0;
};

#endif