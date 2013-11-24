#ifndef ESPSUBPOPULATION_H
#define ESPSUBPOPULATION_H

#include "espchromosome.h"

struct ESPParameters
{
    uint populationSize;
    uint maxGenerations;
    uint sampleEvaluationsPerChromosome;

    string nnFormatFilename;
    double stagnationThreshold;
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
    ESPSubPopulation(ESPParameters _parameters);
    ~ESPSubPopulation();
    ESPSubPopulation(const ESPSubPopulation& _other);
    ESPSubPopulation& operator = (const ESPSubPopulation& _other);

    void generateOffspring();
    void nextGeneration();
    ESPChromosome* getUnevaluatedChromosome();
    ESPChromosome* getChromosome(uint _position);
    void setChromosomeFitness(ESPChromosome* _chromosome, double _fitnessVal);

    void generateDeltaCodes();
    vector<double> getDeltaCode(uint& _pos, bool& _complete);
    void setDeltaCodeFitness(uint _pos, double _fitnessVal);
    void integrateDeltaCodes();

private:
    vector<Chromosome*> mUnevaluatedSubpopulation;
    vector<Chromosome*> mSubpopulation;
    vector<uint> mEvaluationCounter;
    vector<vector<double>> mDeltaCodes;
    vector<uint> mDeltaEvaluationCounter;
    ESPParameters mParameters;
    bool mEvaluationCompleted;
};

#endif