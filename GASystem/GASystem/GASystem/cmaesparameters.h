#ifndef CMAESPARAMETERS_H
#define CMAESPARAMETERS_H

struct CMAESParameters
{
    uint populationSize;
    uint maxGenerations;
    uint maxCompGenerations;

    uint evalsPerCompChrom;

    string nnFormatFilename;
    double fitnessEpsilonThreshold;
    double deltaCodeRadius;

    string selectionAlgorithm;
};


#endif