#ifndef STANDARDGAPARAMETERS_H
#define STANDARDGAPARAMETERS_H

#include "common.h"
#include <string>
#include <map>

struct StandardGAParameters
{
    uint populationSize;
    uint maxGenerations;

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

#endif