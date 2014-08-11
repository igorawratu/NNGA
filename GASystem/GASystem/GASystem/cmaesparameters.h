#ifndef CMAESPARAMETERS_H
#define CMAESPARAMETERS_H

#include <Eigen/Dense>

struct CMAESParameters
{
    //set
    uint maxGenerations;
    uint maxCompGenerations;
    uint evalsPerCompChrom;
    string nnFormatFilename;
    double fitnessEpsilonThreshold;
    double deltaCodeRadius;
    double initStepsize;

    //do not set
    uint populationSize;
    uint parentSize;
    Eigen::MatrixXd weights;
    double mewEff;
};


#endif