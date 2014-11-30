#include "neat.h"

using namespace std;

NEAT::NEAT(NEATParameters _parameters, string _fileName){
    mParameters = _parameters;

	MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    assert(mTotalSlaveProcs > 0);

    int totalWork = _parameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
	mUpdateList = new int[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests * 2];

    for(uint k = 0; k < mTotalRequests * 2; ++k)
        mRetrievedFitnesses[k] = 0;

    mWorkStatus = NOWORK;

    pBestOverallFit = 999999999;
    pNumFitEval = 0;
    pFileName = _fileName;
}

NEAT::~NEAT(){
    delete [] mRequests;
	delete [] mUpdateList;
	delete [] mRetrievedFitnesses;
}

Solution NEAT::train(SimulationContainer* _simulationContainer, string _outputFileName){
    mInnovation = 0;
    mMutationList.clear();
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return Solution(vector<NeuralNetwork>());
    }

    pugi::xml_node root = doc.first_child();

    for(uint k = 0; k < mParameters.populationSize; k++){
        Chromosome* currChrom = new NeatChromosome();
        dynamic_cast<NeatChromosome*>(currChrom)->setGlobalDat(&mInnovation, &mMutationList);

        if(!currChrom->initialize(&root)){
            cerr << "Error: unable to create chromosome in GA" << endl;
            return Solution(vector<NeuralNetwork>());
        }
        else{
            mPopulation.push_back(currChrom);
        }
    }
    mSpecies.push_back(mPopulation);

    mSimulationContainer = _simulationContainer;

    Crossover* crossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    Selection* selectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

	boost::thread workerThread(boost::bind(&NEAT::hostwork, this));

	evaluatePopulation(mPopulation);
    
    if(mTotalRequests > mParameters.populationSize - mParameters.elitismCount)
        mTotalRequests = mParameters.populationSize - mParameters.elitismCount;

    quicksort(mPopulation, 0, mPopulation.size() - 1);
    for(uint i = 0; i < mSpecies.size(); ++i)
        quicksort(mSpecies[i], 0, mSpecies[i].size() - 1);

    uint stagnationCounter = 0;
    for(uint k = 0; k < mParameters.maxGenerations; k++){
        time_t t = time(0);
        cout << "Generation: " << k << endl;

        //create and speciate offspring
        cout << "Creating offspring..." << endl;
        vector<vector<Chromosome*>> offspringSpecies;
        vector<Chromosome*> offspring;
        vector<Chromosome*> representatives;

        createOffspring(offspring, crossoverAlgorithm, selectionAlgorithm);

        //mutate
        cout << "Mutating offspring..." << endl;
        mMutationList.clear();
        for(uint i = 0; i < offspring.size(); ++i){
            dynamic_cast<NeatChromosome*>(offspring[i])->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);
        }

        offspring.insert(offspring.end(), mPopulation.begin(), mPopulation.begin() + mParameters.elitismCount);
        
        //speciation
        cout << "Speciating..." << endl;
        for(uint i = 0; i < mSpecies.size(); ++i){
            representatives.push_back(mSpecies[i][rand() % mSpecies[i].size()]);
            offspringSpecies.push_back(vector<Chromosome*>());
        }

        for(uint i = 0; i < offspring.size(); ++i){
            dynamic_cast<NeatChromosome*>(offspring[i])->setGlobalDat(&mInnovation, &mMutationList);
            bool found = false;
            for(uint l = 0; l < representatives.size(); ++l){
                if(dynamic_cast<NeatChromosome*>(offspring[i])->calcCompDistance(dynamic_cast<NeatChromosome*>(representatives[l])) < mParameters.compatibilityThreshold){
                    found = true;
                    offspringSpecies[l].push_back(offspring[i]);
                    break;
                }
            }

            if(!found){
                vector<Chromosome*> newSpecies;
                newSpecies.push_back(offspring[i]);
                offspringSpecies.push_back(newSpecies);

                representatives.push_back(offspring[i]);
            }
        }

        //remove empty species
        uint l = 0;
        for(uint i = 0; i < offspringSpecies.size(); ++i){
            if(offspringSpecies[i].size() > 0)
                offspringSpecies[l++] = offspringSpecies[i];
        }
        offspringSpecies.resize(l);
        
        //evaluate
        cout << "Evaluating offspring..." << endl;
        evaluatePopulation(offspring);

        cout << "Merging population..." << endl;

        for(uint i = mParameters.elitismCount; i < mPopulation.size(); ++i)
            delete mPopulation[i];

        mPopulation = offspring;
        mSpecies = offspringSpecies;
        cout << mSpecies.size() << endl;
        quicksort(mPopulation, 0, mPopulation.size() - 1);
        for(uint i = 0; i < mSpecies.size(); ++i)
            quicksort(mSpecies[i], 0, mSpecies[i].size() - 1);

        cout << "Current population fitnesses..." << endl;
        for(uint i = 0; i < mPopulation.size(); i++)
            cout << mPopulation[i]->fitness() << " " << mPopulation[i]->realFitness() << " | ";
        cout << endl;

        //checks if the fitness of the solution is below the epsilon threshold, if it is, stop training
        if(mPopulation[0]->realFitness() <= mParameters.fitnessEpsilonThreshold || pNumFitEval => pTotalFitnessEvals){
			mWorkStatus = COMPLETE;
			workerThread.join();

            Solution finalSolution(dynamic_cast<NeatChromosome*>(mPopulation[0])->getNets());
            mSimulationContainer->runFullSimulation(&finalSolution);
	        mSimulationContainer->resetSimulation();

            for(uint j = 0; j < mPopulation.size(); ++j)
                delete mPopulation[j];
			mPopulation.clear();

            delete crossoverAlgorithm;
            delete selectionAlgorithm;

            return finalSolution;
        }

        cout << "Time taken for this generation : " << time(0) - t << endl;

        selectionAlgorithm->tick();
    }

	mWorkStatus = COMPLETE;
	workerThread.join();

    quicksort(mPopulation, 0, mPopulation.size() - 1);
	Solution finalSolution(dynamic_cast<NeatChromosome*>(mPopulation[0])->getNets());
	mSimulationContainer->runFullSimulation(&finalSolution);
	mSimulationContainer->resetSimulation();

    for(uint i = 0; i < mPopulation.size(); i++)
        delete mPopulation[i];
	mPopulation.clear();

    delete crossoverAlgorithm;
    delete selectionAlgorithm;

    mSimulationContainer = 0;

    return finalSolution;
}

void NEAT::createOffspring(vector<Chromosome*>& _offspring, Crossover* _coAlg, Selection* _selAlg){
    quicksort(mPopulation, 0, mPopulation.size() - 1);
    for(uint k = 0; k < mParameters.elitismCount; ++k){
        _offspring.push_back(mPopulation[k]->clone());
    }

    vector<pair<int, double>> avgFitnesses;
    for(uint k = 0; k < mSpecies.size(); ++k){
        double avg = 0;
        for(uint i = 0; i < mSpecies[k].size(); ++i){
            avg += mSpecies[k][i]->fitness();
        }

        avg /= mSpecies[k].size();
        avgFitnesses.push_back(make_pair(k, avg));
    }

    quicksortRanks(avgFitnesses, 0, avgFitnesses.size() - 1);

    double max = (mSpecies.size() * (mSpecies.size() + 1))/2;

    for(uint k = 0; k < avgFitnesses.size(); ++k){
        double specOffNum = boost::math::round(((double)(avgFitnesses.size() - k) / max) * (double)(mParameters.populationSize - mParameters.elitismCount));
        vector<Chromosome*> offspring = _coAlg->execute(mSpecies[avgFitnesses[k].first], (uint)specOffNum, mParameters.crossoverParameters, _selAlg);
        _offspring.insert(_offspring.end(), offspring.begin(), offspring.end());
    }

}

void NEAT::quicksort(vector<Chromosome*>& elements, int left, int right){
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

void NEAT::quicksortRanks(vector<pair<int, double>>& elements, int left, int right){
    int i = left;
	int j = right;

	pair<int, double> pivot = elements[(left+ right) / 2];
	do{
		while (elements[i].second < pivot.second)
			i++;
		while (elements[j].second > pivot.second)
			j--;

		if (i <= j){
			pair<int, double> temp = elements[i]; elements[i] = elements[j]; elements[j] = temp;
			i++; j--;
		}
	}while (i <= j);

	if(left < j)
		quicksortRanks(elements, left, j);
	if(i < right)
		quicksortRanks(elements, i, right);
}

void NEAT::hostwork(){
    while(mWorkStatus != COMPLETE){
        if(mWorkStatus == WORK){
            mSimulationContainer->runFullSimulation(mSavedSolution);
            mSimulationContainer->resetSimulation();

            mRetrievedFitnesses[0] = mSavedSolution->realFitness();
            mRetrievedFitnesses[1] = mSavedSolution->fitness();

            delete mSavedSolution;

            mWorkStatus = NOWORK;
        }
    }
}

void NEAT::stopSlaves(){
    for(uint k = 1; k < mTotalSlaveProcs; ++k){
        int stopmsg[3] = {-1, -1, -1};
        MPI_Send(&stopmsg[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
    }
}

void NEAT::sendData(Solution* _solution, int _slave){
    if(_slave == 0){
        mSavedSolution = _solution;
        mWorkStatus = WORK;
    }
    else{
        //construct serialized data to send through
        int initialDat[4];
        int *nodes, *format;
        double* weights;

        _solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);
        initialDat[3] = 1;

        MPI_Send(&initialDat[0], 4, MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(nodes, initialDat[0], MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(format, initialDat[1], MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(weights, initialDat[2], MPI_DOUBLE, _slave, 1, MPI_COMM_WORLD);

        //setup corresponding nonblocking receive call
        MPI_Irecv(&mRetrievedFitnesses[_slave * 2], 2, MPI_DOUBLE, _slave, 1, MPI_COMM_WORLD, &mRequests[_slave]);

        delete [] nodes;
        delete [] format;
        delete [] weights;
        delete _solution;
    }
}

void NEAT::evaluatePopulation(vector<Chromosome*>& _population){
    uint currPos = 0;

	for(currPos = 0; currPos < mTotalRequests; ++currPos){
		Solution* currSolution = new Solution(dynamic_cast<NeatChromosome*>(_population[currPos])->getNets());
		sendData(currSolution, currPos);
		mUpdateList[currPos] = currPos;
	}

	while(currPos < _population.size()){
		Solution* currSolution = new Solution(dynamic_cast<NeatChromosome*>(_population[currPos])->getNets());
		bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                if(k == 0 && mWorkStatus == NOWORK)
                    assigned = true;
                else if(k > 0){
                    MPI_Status status;
                    int received;

                    MPI_Test(&mRequests[k], &received, &status);

                    if(received)
                        assigned = true;
                }
                if(assigned){
                    _population[mUpdateList[k]]->realFitness() = mRetrievedFitnesses[k*2];
					_population[mUpdateList[k]]->fitness() = mRetrievedFitnesses[k*2 + 1];

                    if(mRetrievedFitnesses[k*2 + 1] < pBestOverallFit)
                        pBestOverallFit = mRetrievedFitnesses[k*2 + 1];

                    pNumFitEval++;
                    if(pNumFitEval > 0 && pNumFitEval % 10 == 0)
                        FileWriter::writeToFile(pFileName, boost::lexical_cast<string>(pBestOverallFit));

                    mRetrievedFitnesses[k*2] = 0;
                    mRetrievedFitnesses[k*2 + 1] = 0;

					mUpdateList[k] = currPos++;

                    sendData(currSolution, k);

                    break;
                }
            }
        }
	}

	for(int k = 0; k < mTotalRequests; ++k){
        bool completed = false;
        //poll the slave until it is complete
        while(!completed){
            Sleep(10);
            if(k == 0 && mWorkStatus == NOWORK)
                completed = true;
            else if (k > 0){
                MPI_Status status;
                int received;

                MPI_Test(&mRequests[k], &received, &status);

                if(received)
                    completed = true;
            }

			if(completed){
                _population[mUpdateList[k]]->realFitness() = mRetrievedFitnesses[k*2];
				_population[mUpdateList[k]]->fitness() = mRetrievedFitnesses[k*2 + 1];

                if(mRetrievedFitnesses[k*2 + 1] < pBestOverallFit)
                    pBestOverallFit = mRetrievedFitnesses[k*2 + 1];

                pNumFitEval++;
                if(pNumFitEval > 0 && pNumFitEval % 10 == 0)
                    FileWriter::writeToFile(pFileName, boost::lexical_cast<string>(pBestOverallFit));
			}
        }
    }
}