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

protected:
    bool checkParams(map<string, vector3> _pos, map<string, double> _dblAcc, map<string, long> _intAcc){
        bool paramsFine = true;

        for(uint k = 0; k < mDoubleStrings.size(); ++k){
            double val;
            paramsFine = getParameter<double>(_dblAcc, val, mDoubleStrings[k]);
        }

        for(uint k = 0; k < mIntStrings.size(); ++k){
            long val;
            paramsFine = getParameter<long>(_intAcc, val, mIntStrings[k]);
        }

        for(uint k = 0; k < mVectorStrings.size(); ++k){
            vector3 val;
            paramsFine = getParameter<vector3>(_pos, val, mVectorStrings[k]);
        }

        return paramsFine;
    }

protected:
    vector<string> mDoubleStrings, mIntStrings, mVectorStrings;
};

#endif