#include "algorithmcreator.h"

Selection* AlgorithmCreator::createSelectionAlgorithm(string _algorithmName){
    return mSelectionFactory.create(_algorithmName);
}

    
Mutation* AlgorithmCreator::createMutationAlgorithm(string _algorithmName){
    return mMutationFactory.create(_algorithmName);
}

Crossover* AlgorithmCreator::createCrossoverAlgorithm(string _algorithmName){
    return mCrossoverFactory.create(_algorithmName);
}


void AlgorithmCreator::startup(){
    AlgorithmCreator::initialized = true;

    AlgorithmCreator::algorithmCreator.startupSelectionAlgorithms();
    AlgorithmCreator::algorithmCreator.startupMutationAlgorithms();
    AlgorithmCreator::algorithmCreator.startupCrossoverAlgorithms();
}

void AlgorithmCreator::shutdown(){
    AlgorithmCreator::algorithmCreator.mMutationFactory.clear();
    AlgorithmCreator::algorithmCreator.mSelectionFactory.clear();
    AlgorithmCreator::algorithmCreator.mCrossoverFactory.clear();

    AlgorithmCreator::initialized = false;
}


void AlgorithmCreator::startupSelectionAlgorithms(){
    
}

void AlgorithmCreator::startupMutationAlgorithms(){

}

void AlgorithmCreator::startupCrossoverAlgorithms(){

}