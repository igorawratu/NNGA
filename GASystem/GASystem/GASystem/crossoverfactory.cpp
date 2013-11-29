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
    //crossoverFactory.mFactory.registerCreator("SPX", SPX::createSPX);
    crossoverFactory.mFactory.registerCreator("BLX", BLX::createBLX);
    crossoverFactory.mFactory.registerCreator("UNDX", UNDX::createUNDX);
    crossoverFactory.mFactory.registerCreator("PCX", PCX::createPCX);
}

void CrossoverFactory::shutdown(){
    crossoverFactory.mFactory.clear();
    initialized = false;
}