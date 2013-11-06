#ifndef BOLTZMANNSELECTION_H
#define BOLTZMANNSELECTION_H

#include "selection.h"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <vector>
#include <math.h> 

using namespace std;

class BoltzmannSelection : public Selection
{
public:
    BoltzmannSelection();
    virtual ~BoltzmannSelection();

    static Selection* createBoltzmannSelection(){
        return new BoltzmannSelection();
    }

    virtual vector<Chromosome*> execute(vector<Chromosome*> _selectionPool, uint _selectionCount, vector<Chromosome*>& _unselected);

private:
    Chromosome* selectChromosome(vector<Chromosome*>& _population);

    void calculateFitness(vector<Chromosome*> _selectionPool);

    void calculateMaxFitness();

private:
    double mTemperature, mInitTemp, mMaxFit;
    int mMaxGenerations;
    vector<double> mFitnessVals;
};

#endif