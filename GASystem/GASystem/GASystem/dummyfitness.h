#ifndef DUMMYFITNESS_H
#define DUMMYFITNESS_H

class DummyFitness
{
public:
    DummyFitness(){}
    virtual ~DummyFitness(){}

    virtual double evaluateFitness(vector<vector3> _pos, map<uint, double> _dblAcc, map<uint, long> _intAcc){
        return 0;
    }
};

#endif