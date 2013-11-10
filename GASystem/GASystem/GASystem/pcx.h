#ifndef PCX_H
#define PCX_H

#include "crossover.h"
#include <math.h>

class PCX : public Crossover
{
public:
    PCX();
    virtual ~PCX();

    virtual vector<Chromosome*> execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters);

    static Crossover* createPCX(){
        return new PCX();
    }

private:
    void calculateDMParents(const vector<Chromosome*>& _parents, vector<double>& _d, vector<double>& _m, vector<double>& _p1, vector<double>& _p2, vector<double>& p3);

    double calculateP3DDistance(const vector<double>& _dvec, const vector<double>& _p1vec, const vector<double>& _p3vec);

    void calculateOrthogonalBasis(double _spanvecdim, double _spansize, double** _initialspan, double** _orthobasisvectors, const vector<double>& _dvec);

};

#endif