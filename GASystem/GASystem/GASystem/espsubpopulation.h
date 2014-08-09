#ifndef ESPSUBPOPULATION_H
#define ESPSUBPOPULATION_H

#include "espchromosome.h"
#include "selectionfactory.h"
#include "crossoverfactory.h"
#include "crossover.h"
#include "selection.h"
#include "pugixml.hpp"
#include "neuron.h"
#include "espparameters.h"
#include <iostream>

using namespace std;

class ESPSubPopulation
{
public:
    ESPSubPopulation(ESPParameters _parameters, pugi::xml_node* _root, uint _teamID);
    ~ESPSubPopulation();
    ESPSubPopulation(const ESPSubPopulation& _other);
    ESPSubPopulation& operator = (const ESPSubPopulation& _other);

    void generateOffspring();
    void nextGeneration();
    Chromosome* getUnevaluatedChromosome();
    Chromosome* getChromosome(uint _position);
    void setChromosomeFitness(Neuron* _neuron, double _fitnessVal, double _realFitnessVal);
    void print();

    void generateDeltaCodes(double _deltaRange);

    void setParameters(ESPParameters _params);

    void printBestWorstDistance();

    uint getTeamID();
    void setTeamID(uint _id);

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right);

private:
    vector<Chromosome*> mUnevaluatedSubpopulation;
    vector<Chromosome*> mSubpopulation;
    vector<uint> mEvaluationCounter;
    ESPParameters mParameters;
    Crossover* mCrossoverAlgorithm;
    Selection* mSelectionAlgorithm;

    uint mTeamID;

    double mLastDCRadius;
    double mBestLastFitness;
    uint mStagnationCounter;
};

#endif