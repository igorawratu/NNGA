#include "crossoverfactory.h"

CrossoverFactory CrossoverFactory::crossoverFactory;
bool CrossoverFactory::initialized;

CrossoverFactory& CrossoverFactory::instance(){
    if(!initialized)
        crossoverFactory.startup();

    return crossoverFactory;
}

Crossover* CrossoverFactory::create(string _algorithmName){
    return mFactory.create(_algorithmName);
}

void CrossoverFactory::startup(){
    initialized = true;

    crossoverFactory.mFactory.registerCreator("MultipointCrossover", MultipointCrossover::createMultipointCrossover);
    crossoverFactory.mFactory.registerCreator("SPX", SPX::createSPX);
}

void CrossoverFactory::shutdown(){
    crossoverFactory.mFactory.clear();
    initialized = false;
}