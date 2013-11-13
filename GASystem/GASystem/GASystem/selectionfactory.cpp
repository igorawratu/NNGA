#include "selectionfactory.h"

SelectionFactory SelectionFactory::selectionFactory;
bool SelectionFactory::initialized;

SelectionFactory& SelectionFactory::instance(){
    if(!initialized)
        selectionFactory.startup();

    return selectionFactory;
}

Selection* SelectionFactory::create(string _algorithmName){
    return mFactory.create(_algorithmName);
}

void SelectionFactory::startup(){
    initialized = true;

    selectionFactory.mFactory.registerCreator("TournamentSelection", TournamentSelection::createTournamentSelection);
    selectionFactory.mFactory.registerCreator("QuadraticRankSelection", QuadraticRankSelection::createQuadraticRankSelection);
    selectionFactory.mFactory.registerCreator("LRankSelection", LRankSelection::createLRankSelection);
    selectionFactory.mFactory.registerCreator("NLRankSelection", NLRankSelection::createNLRankSelection);
    selectionFactory.mFactory.registerCreator("BoltzmannSelection", BoltzmannSelection::createBoltzmannSelection);
    selectionFactory.mFactory.registerCreator("RandomSelection", RandomSelection::createRandomSelection);
}

void SelectionFactory::shutdown(){
    initialized = false;

    selectionFactory.mFactory.clear();
}