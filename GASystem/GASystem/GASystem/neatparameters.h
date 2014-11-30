#ifndef NEATPARAMETERS_H
#define NEATPARAMETERS_H

#include "common.h"
#include <string>
#include <map>

struct NEATParameters
{
    uint populationSize;
    uint maxGenerations;

    string nnFormatFilename;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

    uint elitismCount;
    double compatibilityThreshold;
};

#endif