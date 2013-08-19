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

    virtual double evaluateFitness(vector<vector3> _pos, map<uint, double> _dblAcc, map<uint, long> _intAcc)=0;
};

#endif