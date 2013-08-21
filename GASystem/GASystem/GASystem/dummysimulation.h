#ifndef DUMMYSIMULATION_H
#define DUMMYSIMULATION_H

#include "common.h"
#include "simulation.h"

#include <iostream>

using namespace std;

class DummySimulation : public Simulation
{
public:
    DummySimulation(uint _numCycles, uint _cyclesPerDecision) : Simulation(_numCycles, _cyclesPerDecision){}
    virtual ~DummySimulation(){}

    virtual void iterate(){}
    virtual void render(){cerr << "Error: cannot render a dummy simulation" << endl;}
    virtual double fitness(vector<Fitness*> _fit){
        double finalFitness = 0;
        map<uint, double> dblAcc;
        map<uint, long> intAcc;
        vector<vector3> pos;

        for(uint k = 0; k < _fit.size(); k++)
            finalFitness += _fit[k]->evaluateFitness(pos, dblAcc, intAcc);

        return finalFitness;
    }

    virtual Simulation* getNewCopy(){
        return new DummySimulation(mNumCycles, mCyclesPerDecision);
    }

    virtual bool initialise(ResourceManager* _rm){
        return true;
    }

};

#endif