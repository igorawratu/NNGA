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

    selectionFactory.mFactory.registerCreator("RankSelection", RankSelection::createRankSelection);
    selectionFactory.mFactory.registerCreator("RankExponentialSelection", RankExponentialSelection::createRankExponentialSelection);
    selectionFactory.mFactory.registerCreator("TournamentSelection", TournamentSelection::createTournamentSelection);
    selectionFactory.mFactory.registerCreator("RankQuadraticSelection", RankQuadraticSelection::createRankQuadraticSelection);
}

void SelectionFactory::shutdown(){
    initialized = false;

    selectionFactory.mFactory.clear();
}