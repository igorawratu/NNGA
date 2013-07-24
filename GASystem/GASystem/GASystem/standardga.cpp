#include "standardga.h"

StandardGA::StandardGA(StandardGAParameters _parameters){

}


StandardGA::StandardGA(const StandardGA& other){

}


StandardGA& StandardGA::operator = (const StandardGA& other){

}


StandardGA::~StandardGA(){

}

struct StandardGAParameters
{
    uint populationSize;
    uint maxGenerations;

    string nnFormatFilename;
    double stagnationMovementThreshold;
    double fitnessEpsilonThreshold;

    string mutationAlgorithm;
    map<string, double> mutationParameters;
    string crossoverAlgorithm;
    map<string, double> crossoverParameters;
    string selectionAlgorithm;

};

Solution StandardGA::train(SimulationContainer* _simulationContainer){
    vector<Chromosome*> population;
    for(uint k = 0; k < mParameters.populationSize; k++)
        population.push_back(new NNChromosome(mParameters.nnFormatFilename));

    Crossover* crossoverAlgorithm = AlgorithmCreator::instance().createCrossoverAlgorithm(mParameters.crossoverAlgorithm);
    Selection* selectionAlgorithm = AlgorithmCreator::instance().createSelectionAlgorithm(mParameters.selectionAlgorithm);
    
    for(uint i = 0; i < population.size(); i++){
        Solution currSolution(dynamic_cast<NNChromosome*>(population[i])->getNeuralNets());
        _simulationContainer->runFullSimulation(&currSolution);
        population[i]->fitness() = currSolution.fitness();
    }

    for(uint k = 0; k < mParameters.maxGenerations; k++){
        //sort population here

        //create offspring
        vector<Chromosome*> offspring = crossoverAlgorithm->execute(population, population.size(), mParameters.crossoverParameters);
        
        //mutate offspring
        for(uint i = 0; i < offspring.size(); i++)
            offspring[i]->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);

        //evaluate offspring
        for(uint i = 0; i < offspring.size(); i++){
            Solution currSolution(dynamic_cast<NNChromosome*>(offspring[i])->getNeuralNets());
            _simulationContainer->runFullSimulation(&currSolution);

            //checks if the fitness of the solution is below the epsilon threshold, if it is, stop training
            if(currSolution.fitness() < mParameters.fitnessEpsilonThreshold){
                for(uint i = 0; i < population.size(); i++)
                    delete population[i];
                for(uint i = 0; i < population.size(); i++)
                    delete population[i];
                return currSolution;
            }

            offspring[i]->fitness() = currSolution.fitness();
        }

        //parent child contention for now
        population.insert(population.end(), offspring.begin(), offspring.end());
        //sort this
        vector<Chromosome*> unselected;
        population = selectionAlgorithm->execute(population, mParameters.populationSize, unselected);
        //delete chromosomes which have not been selected
        for(uint i = 0; i < unselected.size(); i++)
            delete unselected[i];
        unselected.clear();

        //check for stagnation, if none go next generation

    }
}