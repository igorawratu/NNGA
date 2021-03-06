#include "mutationfactory.h"

MutationFactory MutationFactory::mutationFactory;
bool MutationFactory::initialized;

MutationFactory& MutationFactory::instance(){
    if(!initialized)
        mutationFactory.startup();

    return mutationFactory;
}

Mutation* MutationFactory::create(string _algorithmName){
    return mFactory.create(_algorithmName);
}

void MutationFactory::startup(){
    initialized = true;

    mutationFactory.mFactory.registerCreator("GaussianMutation", GaussianMutation::createGaussianMutation);
    mutationFactory.mFactory.registerCreator("CauchyLorentzMutation", CauchyLorentzMutation::createCauchyLorentzMutation);
    mutationFactory.mFactory.registerCreator("UniformMutation", UniformMutation::createUniformMutation);
}

void MutationFactory::shutdown(){
    initialized = false;

    mutationFactory.mFactory.clear();
}