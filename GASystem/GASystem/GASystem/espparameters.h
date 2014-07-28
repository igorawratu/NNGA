#ifndef ESPPARAMETERS_H
#define ESPPARAMETERS_H

#include "common.h"

#include <string>
#include <map>

struct ESPParameters
{
    uint populationSize;
    uint maxGenerations;
    uint maxCompGenerations;
    uint sampleEvaluationsPerChromosome;
    uint stagnationThreshold;

    string nnFormatFilename;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

    double deltaCodeRadius;

    uint elitismCount;
};

#endif