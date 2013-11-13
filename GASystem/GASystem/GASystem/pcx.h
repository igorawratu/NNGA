#ifndef PCX_H
#define PCX_H

#include "crossover.h"
#include <math.h>

class PCX : public Crossover
{
public:
    PCX();
    virtual ~PCX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm);

    static Crossover* createPCX(){
        return new PCX();
    }

private:
    void calculateDGParents(const vector<Chromosome*>& _parents, vector<double>& _d, vector<double>& _g, vector<double>& _p1, vector<double>& _p2, vector<double>& _p3);

    double PCX::calculateMeanDistanceD(const vector<double>& _dvec, const vector<double>& _g, const vector<double>& _p2vec, const vector<double>& _p3vec);

    void PCX::calculateOrthogonalBasis(double _spanvecdim, double _spansize, double** _initialspan, double** _orthbasisvectors, const vector<double>& _dvec);

};

#endif