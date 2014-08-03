#include "standardga.h"

StandardGA::StandardGA(StandardGAParameters _parameters){
    mParameters = _parameters;

	MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    assert(mTotalSlaveProcs > 0);

    int totalWork = _parameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
	mUpdateList = new int[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests * 2];
}


StandardGA::StandardGA(const StandardGA& other){
    mParameters = other.mParameters;
}


StandardGA& StandardGA::operator = (const StandardGA& other){
    mParameters = other.mParameters;

    return *this;
}


StandardGA::~StandardGA(){
	delete [] mRequests;
	delete [] mUpdateList;
	delete [] mRetrievedFitnesses;
}

Solution StandardGA::train(SimulationContainer* _simulationContainer, string _outputFileName){   
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
        else mPopulation.push_back(currChrom);
    }

    mSimulationContainer = _simulationContainer;

    Crossover* crossoverAlgorithm = CrossoverFactory::instance().create(mParameters.crossoverAlgorithm);
    Selection* selectionAlgorithm = SelectionFactory::instance().create(mParameters.selectionAlgorithm);

	boost::thread workerThread(boost::bind(&StandardGA::hostwork, this));
    
	evaluatePopulation(mPopulation);

    //check if total requests exceeds the number of total work required per generation
    if(mTotalRequests > mParameters.populationSize - mParameters.elitismCount)
        mTotalRequests = mParameters.populationSize - mParameters.elitismCount;

    uint stagnationCounter = 0;
    for(uint k = 0; k < mParameters.maxGenerations; k++){
        time_t t = time(0);
        cout << "Generation: " << k << endl;

        quicksort(mPopulation, 0, mPopulation.size() - 1);

        cout << "Creating offspring..." << endl;

        //create offspring
        vector<Chromosome*> offspring = crossoverAlgorithm->execute(mPopulation, mPopulation.size() - mParameters.elitismCount, mParameters.crossoverParameters, selectionAlgorithm);
        
        cout << "Mutating offspring..." << endl;
        //mutate offspring
        for(uint i = 0; i < offspring.size(); i++)
            offspring[i]->mutate(mParameters.mutationAlgorithm, mParameters.mutationParameters);

        cout << "Evaluating offspring..." << endl;
        //evaluate offspring
        evaluatePopulation(offspring);

        cout << "Merging population..." << endl;

        quicksort(mPopulation, 0, mPopulation.size() - 1);
        for(uint k = mParameters.elitismCount; k < mPopulation.size(); ++k)
            delete mPopulation[k];
        mPopulation.erase(mPopulation.begin() + mParameters.elitismCount, mPopulation.end());
        mPopulation.insert(mPopulation.end(), offspring.begin(), offspring.end());
        quicksort(mPopulation, 0, mPopulation.size() - 1);

        cout << "Current population fitnesses..." << endl;
        for(uint i = 0; i < mPopulation.size(); i++)
            cout << mPopulation[i]->fitness() << " " << mPopulation[i]->realFitness() << " | ";
        cout << endl;

        //checks if the fitness of the solution is below the epsilon threshold, if it is, stop training
        for(uint i = 0; i < mPopulation.size(); ++i){
            if(mPopulation[i]->realFitness() <= mParameters.fitnessEpsilonThreshold){
                Solution finalSolution(dynamic_cast<NNChromosome*>(mPopulation[i])->getNeuralNets());
                finalSolution.fitness() = mPopulation[i]->fitness();

				mWorkStatus = COMPLETE;
				stopSlaves();
				workerThread.join();

                for(uint j = 0; j < mPopulation.size(); ++j)
                    delete mPopulation[j];
				mPopulation.clear();

                delete crossoverAlgorithm;
                delete selectionAlgorithm;

                return finalSolution;
            }
        }

        cout << "Time taken for this generation : " << time(0) - t << endl;

        selectionAlgorithm->tick();
    }

	mWorkStatus = COMPLETE;
	stopSlaves();
	workerThread.join();

    quicksort(mPopulation, 0, mPopulation.size() - 1);

	cout << "final population fitnesses..." << endl;
	for(uint i = 0; i < mPopulation.size(); i++){
		Solution sol(dynamic_cast<NNChromosome*>(mPopulation[i])->getNeuralNets());
		mSimulationContainer->runFullSimulation(&sol);
		mSimulationContainer->resetSimulation();
        cout << sol.fitness() << " " << sol.realFitness() << " | ";
	}
    cout << endl;

	Solution finalSolution(dynamic_cast<NNChromosome*>(mPopulation[0])->getNeuralNets());
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

void StandardGA::evaluatePopulation(vector<Chromosome*>& _population){
	uint currPos = 0;

	for(currPos = 0; currPos < mTotalRequests - 1; ++currPos){
		Solution currSolution(dynamic_cast<NNChromosome*>(_population[currPos])->getNeuralNets());
		sendData(currSolution, currPos + 1);
		mUpdateList[currPos + 1] = currPos;
	}

	while(currPos < _population.size()){
		Solution currSolution(dynamic_cast<NNChromosome*>(_population[currPos])->getNeuralNets());
		bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 1; k < mTotalRequests; ++k){
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

					mUpdateList[k] = currPos++;

                    sendData(currSolution, k);

                    break;
                }
            }
        }
	}


	for(int k = 1; k < mTotalRequests; ++k){
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
			}
        }
    }
}

void StandardGA::sendData(Solution& _solution, int _slave){
	if(_slave == 0)
        mWorkStatus = WORK;
    else{
        //construct serialized data to send through
        int initialDat[4];
        int *nodes, *format;
        double* weights;

        _solution.serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);
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
    }
}

void StandardGA::stopSlaves(){
    for(uint k = 1; k < mTotalSlaveProcs; ++k){
        int stopmsg[3] = {-1, -1, -1};
        MPI_Send(&stopmsg[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
    }
}

void StandardGA::hostwork(){
    while(mWorkStatus != COMPLETE){
        if(mWorkStatus == WORK){
            Solution* solution = new Solution(dynamic_cast<NNChromosome*>(mPopulation[mUpdateList[0]])->getNeuralNets());
            mSimulationContainer->runFullSimulation(solution);
            mSimulationContainer->resetSimulation();

            mRetrievedFitnesses[0] = solution->realFitness();
            mRetrievedFitnesses[1] = solution->fitness();

            delete solution;

            mWorkStatus = NOWORK;
        }
    }
}