#ifndef ESPSUBPOPULATION_H
#define ESPSUBPOPULATION_H

#include "espchromosome.h"
#include "selectionfactory.h"
#include "crossoverfactory.h"
#include "crossover.h"
#include "selection.h"
#include "pugixml.hpp"
#include "neuron.h"
#include <iostream>

using namespace std;

struct ESPParameters
{
    uint populationSize;
    uint maxGenerations;
    uint sampleEvaluationsPerChromosome;
    uint stagnationThreshold;

    string nnFormatFilename;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

    uint elitismCount;
};

class ESPSubPopulation
{
public:
    ESPSubPopulation(ESPParameters _parameters, pugi::xml_node* _root);
    ~ESPSubPopulation();
    ESPSubPopulation(const ESPSubPopulation& _other);
    ESPSubPopulation& operator = (const ESPSubPopulation& _other);

    void generateOffspring();
    void nextGeneration();
    Chromosome* getUnevaluatedChromosome();
    Chromosome* getChromosome(uint _position);
    void setChromosomeFitness(Neuron* _chromosome, double _fitnessVal, double _realFitnessVal);
    void print();

    /*void generateDeltaCodes();
    vector<double> getDeltaCode(uint& _pos, bool& _complete);
    void setDeltaCodeFitness(uint _pos, double _fitnessVal);
    void integrateDeltaCodes();*/

private:
    void quicksort(vector<Chromosome*>& elements, int left, int right);

private:
    vector<Chromosome*> mUnevaluatedSubpopulation;
    vector<Chromosome*> mSubpopulation;
    vector<uint> mEvaluationCounter;
    ESPParameters mParameters;
    Crossover* mCrossoverAlgorithm;
    Selection* mSelectionAlgorithm;
};

#endif