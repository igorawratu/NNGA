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
    crossoverFactory.mFactory.registerCreator("TPX", TPX::createTPX);
    crossoverFactory.mFactory.registerCreator("AX", AX::createAX);
    crossoverFactory.mFactory.registerCreator("SBX", SBX::createSBX);
    crossoverFactory.mFactory.registerCreator("PCX", OPX::createOPX);
    crossoverFactory.mFactory.registerCreator("HX", HX::createHX);
}

void CrossoverFactory::shutdown(){
    crossoverFactory.mFactory.clear();
    initialized = false;
}