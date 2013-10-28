#include "standardga.h"

StandardGA::StandardGA(StandardGAParameters _parameters){
    mParameters = _parameters;
}


StandardGA::StandardGA(const StandardGA& other){
    mParameters = other.mParameters;
}


StandardGA& StandardGA::operator = (const StandardGA& other){
    mParameters = other.mParameters;

    return *this;
}


StandardGA::~StandardGA(){
}

Solution StandardGA::train(SimulationContainer* _simulationContainer){
    vector<Chromosome*> population;
    
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return Solution(vector<NeuralNetwork>());
    }

    pugi::xml_node root = doc.first_child();

    for(uint k = 0; k < mParameters.populationSize; k++){
        NNChromosome* currChrom = new NNChromosome();
        if(!currChrom->initialize(&root)){
            cerr << "Error: unable to create chromosome in GA" << endl;
            return Solution(vector<NeuralNetwork>());
        }
        else population.push_back(currChrom);
    }

    Crossover* crossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    Selection* selectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);
    
    for(int i = 0; i < population.size(); ++i){
        SimulationContainer* simcont = _simulationContainer->clone();

        Solution currSolution(dynamic_cast<NNChromosome*>(population[i])->getNeuralNets());
        simcont->runFullSimulation(&currSolution);
        population[i]->fitness() = currSolution.fitness();
        population[i]->realFitness() = currSolution.realFitness();

        delete simcont;
    }

    uint stagnationCounter = 0;
    for(uint k = 0; k < mParameters.maxGenerations; k++){
        time_t t = time(0);
        cout << "Generation: " << k << endl;

        quicksort(population, 0, population.size() - 1);

        cout << "Creating offspring..." << endl;

        //create offspring
        vector<Chromosome*> offspring = crossoverAlgorithm->execute(population, population.size(), mParameters.crossoverParameters);
        
        cout << "Mutating offspring..." << endl;
        //mutate offspring
        for(uint i = 0; i < offspring.size(); i++)
            offspring[i]->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);

        cout << "Evaluating offspring..." << endl;
        //evaluate offspring
        for(int i = 0; i < offspring.size(); ++i){
            SimulationContainer* simcont = _simulationContainer->clone();

            Solution currSolution(dynamic_cast<NNChromosome*>(offspring[i])->getNeuralNets());
            simcont->runFullSimulation(&currSolution);
            offspring[i]->fitness() = currSolution.fitness();
            offspring[i]->realFitness() = currSolution.realFitness();

            delete simcont;
        }

        cout << "Merging population..." << endl;
        //merge parents and children into 1 population as they contend with each other
        population.insert(population.end(), offspring.begin(), offspring.end());

        quicksort(population, 0, population.size() - 1);
        cout << "Population before selection..." << endl;
        for(uint i = 0; i < population.size(); i++)
            cout << population[i]->fitness() << " " << population[i]->realFitness() << " | ";
        cout << endl;

        //checks if the fitness of the solution is below the epsilon threshold, if it is, stop training
        for(uint i = 0; i < population.size(); ++i){
            if(population[i]->realFitness() <= mParameters.fitnessEpsilonThreshold){
                Solution finalSolution(dynamic_cast<NNChromosome*>(population[0])->getNeuralNets());
                finalSolution.fitness() = population[i]->fitness();

                for(uint j = 0; j < population.size(); ++j)
                    delete population[j];

                delete crossoverAlgorithm;
                delete selectionAlgorithm;

                return finalSolution;
            }
        }

        vector<Chromosome*> unselected;
        vector<Chromosome*> newPopulation;
        for(uint i = 0; i < mParameters.elitismCount; i++)
            newPopulation.push_back(population[i]);

        for(uint i = 0; i < mParameters.elitismCount; i++)
            population.erase(population.begin());

        population = selectionAlgorithm->execute(population, mParameters.populationSize - mParameters.elitismCount, unselected);

        newPopulation.insert(newPopulation.end(), population.begin(), population.end());
        population = newPopulation;
        
        //delete chromosomes which have not been selected
        for(uint i = 0; i < unselected.size(); i++)
            delete unselected[i];
        unselected.clear();

        quicksort(population, 0, population.size() - 1);
        
        
        cout << "Current population fitnesses..." << endl;
        for(uint i = 0; i < population.size(); i++)
            cout << population[i]->fitness() << " | ";
        cout << endl;


        cout << "Checking termination conditions..." << endl;
        //calculates standard deviation, if it has been below the threshold for 10(arb, can make this var) generations, then stagnation
        double meanFit = 0, variance = 0;
        for(uint i = 0; i < population.size(); i++)
            meanFit += population[i]->fitness();
        meanFit /= population.size();

        for(uint i = 0; i < population.size(); i++){
            double dif = population[i]->fitness() - meanFit;
            variance += dif * dif;
        }
        variance /= population.size();
        if(sqrt(variance) < mParameters.stagnationThreshold)
            stagnationCounter++;
        else stagnationCounter = 0;

        if(stagnationCounter > 10)        
        {
            cout << "Population stagnated" << endl;
            quicksort(population, 0, population.size() - 1);

            Solution finalSolution(dynamic_cast<NNChromosome*>(population[0])->getNeuralNets());

            for(uint i = 0; i < population.size(); i++)
                delete population[i];

            delete crossoverAlgorithm;
            delete selectionAlgorithm;

            return finalSolution;
        }
        cout << "Time taken for this generation : " << time(0) - t << endl;
    }
    quicksort(population, 0, population.size() - 1);

    Solution finalSolution(dynamic_cast<NNChromosome*>(population[0])->getNeuralNets());
    _simulationContainer->runFullSimulation(&finalSolution);
    _simulationContainer->resetSimulation();

    for(uint i = 0; i < population.size(); i++)
        delete population[i];

    delete crossoverAlgorithm;
    delete selectionAlgorithm;

    return finalSolution;
}

void StandardGA::quicksort(vector<Chromosome*>& elements, int left, int right)
{
	int i = left;
	int j = right;

	Chromosome* pivot = elements[(left+ right) / 2];
	do{
		while (elements[i]->fitness() < pivot->fitness())
			i++;
		while (elements[j]->fitness() > pivot->fitness())
			j--;

		if (i <= j){
			Chromosome* temp = elements[i]; elements[i] = elements[j]; elements[j] = temp;
			i++; j--;
		}
	}while (i <= j);

	if(left < j)
		quicksort(elements, left, j);
	if(i < right)
		quicksort(elements, i, right);
}