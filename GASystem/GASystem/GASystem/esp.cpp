#include "esp.h"

ESP::ESP(ESPParameters _parameters){
    mParameters = _parameters;

    setupSubpopulationStructure();

    MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    assert(mTotalSlaveProcs > 0);

    int totalWork = _parameters.sampleEvaluationsPerChromosome * _parameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
    mTeamRequests = new MPI_Request[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests * 2];

    if(mNumTeams > 1){
        mRetrievedCompetitiveFitnesses = new double[mTotalRequests * mNumTeams];
        mRetrievedTeamIDs = new int[mTotalRequests * mNumTeams];
    }

    mWorkStatus = NOWORK;
}

ESP::~ESP(){
    for(uint k = 0; k < mSubpopulations.size(); ++k){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            delete iter->second.first;
        }
    }

    delete[] mRetrievedFitnesses;
    delete[] mRequests;
}

Solution ESP::train(SimulationContainer* _simulationContainer, string _outputFileName){
    mBestFitness = mBestRealFitness = -1;
    mSimulationContainer = _simulationContainer;
    
    //launch host worker thread
    boost::thread workerThread(boost::bind(&ESP::hostwork, this));

    if(mNumTeams == 1 || mParameters.maxCompGenerations == 0)
        mStages = 1;
    else mStages = 2;

    for(mStage = 1; mStage <= mStages; ++mStage){ 
        if(mStage == 2){
            runDeltaCodes();
            mParameters.mutationParameters["MutationProbability"] = 0;

            for(uint i = 0; i < mSubpopulations.size(); ++i)
                for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
                    if(iter->second.first != NULL)
                        iter->second.first->setParameters(mParameters);
                }
        }

        uint gen;

        if(mStages == 1 || mStage == 2){
            evaluateFitness(_simulationContainer);
            gen = mParameters.maxGenerations;
        }
        else{
            evaluateCompetitiveFitness(_simulationContainer);
            gen = mParameters.maxCompGenerations;
        }

        //check if total requests exceeds the number of total work required per generation
        if(mTotalRequests > mParameters.populationSize - mParameters.elitismCount)
            mTotalRequests = mParameters.populationSize - mParameters.elitismCount;

        for(uint k = 0; k < gen; ++k){
            time_t t = time(0);
            
            cout << "Generation " << k << endl;
            cout << "Generating and mutating offspring" << endl;
            for(uint i = 0; i < mSubpopulations.size(); ++i){
                for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
                    if(iter->second.second != 0)
                        iter->second.first->generateOffspring();
                }
            }

            cout << "Evaluating Fitness" << endl;
            if(mStages == 1 || mStage == 2)
                evaluateFitness(_simulationContainer);
            else evaluateCompetitiveFitness(_simulationContainer);

            if(mStages == 1 || mStage == 2)
                cout << "Best Fitness: " << mBestFitness << " - " << mBestRealFitness << endl;

            for(uint i = 0; i < mSubpopulations.size(); ++i){
                for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
                    if(iter->second.second != 0){
                        iter->second.first->nextGeneration();
                        cout << "Subpop " << iter->first << " best-worst distance : "; 
                        iter->second.first->printBestWorstDistance();
                    }
                }
            }

            cout << "Time taken for this generation : " << time(0) - t << endl;

            if(mStages == 1 || mStage == 2){
                if(mBestRealFitness <= mParameters.fitnessEpsilonThreshold){
                    stopSlaves();
                    mWorkStatus = COMPLETE;
                    workerThread.join();
                    return mBestSolution;
                }
            }
        }
    }

    mWorkStatus = COMPLETE;
    workerThread.join();
    stopSlaves();
    return mBestSolution;
}

void ESP::stopSlaves(){
    for(uint k = 1; k < mTotalSlaveProcs; ++k){
        int stopmsg[3] = {-1, -1, -1};
        MPI_Send(&stopmsg[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
    }
}

void ESP::hostwork(){
    while(mWorkStatus != COMPLETE){
        if(mWorkStatus == WORK){
            Solution* solution = mSavedSolutions[0];
            mSimulationContainer->runFullSimulation(solution);
            mSimulationContainer->resetSimulation();

            if(mStages == 1 || mStage == 2){
                mRetrievedFitnesses[0] = solution->realFitness();
                mRetrievedFitnesses[1] = solution->fitness();
            }
            else{
                vector<CompetitiveFitness> compFitnesses = solution->competitiveFitness();
                for(uint k = 0; k < compFitnesses.size(); ++k){
                    mRetrievedTeamIDs[k] = compFitnesses[k].first;
                    mRetrievedCompetitiveFitnesses[k] = compFitnesses[k].second;
                }
            }

            mWorkStatus = NOWORK;
        }
    }
}

void ESP::sendCompData(Solution& _solution, int _slave){
    if(_slave == 0)
        mWorkStatus = WORK;
    else{
        //construct serialized data to send through
        int initialDat[4];
        int *nodes, *format;
        double* weights;

        _solution.serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);
        initialDat[3] = 0;

        MPI_Send(&initialDat[0], 4, MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(nodes, initialDat[0], MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(format, initialDat[1], MPI_INT, _slave, 1, MPI_COMM_WORLD);
        MPI_Send(weights, initialDat[2], MPI_DOUBLE, _slave, 1, MPI_COMM_WORLD);

        //setup corresponding nonblocking receive calls
        MPI_Irecv(&mRetrievedCompetitiveFitnesses[_slave * mNumTeams], mNumTeams, MPI_DOUBLE, _slave, 1, MPI_COMM_WORLD, &mRequests[_slave]);
        MPI_Irecv(&mRetrievedTeamIDs[_slave * mNumTeams], mNumTeams, MPI_INT, _slave, 1, MPI_COMM_WORLD, &mTeamRequests[_slave]);

        delete [] nodes;
        delete [] format;
        delete [] weights;
    }
}

void ESP::sendData(Solution& _solution, int _slave){
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

void ESP::saveUpdateVec(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _nnPrims, int _slave){
    vector<map<uint, Neuron*>> currUpdateVec;
        
    for(uint i = 0; i < _nnPrims.size(); ++i){
        currUpdateVec.push_back(map<uint, Neuron*>());
        for(map<uint, Neuron*>::iterator iter = _nnPrims[i].first.begin(); iter != _nnPrims[i].first.end(); ++iter){
            if(iter->second->getNeuronType() == LEAF)
                delete iter->second;
            else currUpdateVec[i][iter->first] = iter->second;
        }
    }

    mUpdateList[_slave] = currUpdateVec;
}

Solution* ESP::constructSolution(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> _nnPrims){
    vector<NeuralNetwork> neuralNets;

    for(uint i = 0; i < _nnPrims.size(); ++i){
        neuralNets.push_back(NeuralNetwork());
        neuralNets[i].setStructure(_nnPrims[i].first, _nnPrims[i].second);
    }

    return new Solution(neuralNets);
}

void ESP::updateCompFitness(int _slave){
    for(uint i = 0; i < mUpdateList[_slave].size(); ++i){
        for(map<uint, Neuron*>::iterator iter = mUpdateList[_slave][i].begin(); iter != mUpdateList[_slave][i].end(); ++iter){
            uint currFitnessTeamID = mSubpopulations[i][iter->first].first->getTeamID();
            double fitness = 0;
            for(uint i = _slave; i < _slave + mNumTeams; ++i){
                if(mRetrievedTeamIDs[i] == currFitnessTeamID){
                    fitness = mRetrievedCompetitiveFitnesses[i];
                    break;
                }
            }
            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, fitness);
        }
    }
    mUpdateList.erase(_slave);
}

void ESP::updateFitness(int _slave, bool& _improved){
    //update chromosomes with fitness
    double realFitness = mRetrievedFitnesses[_slave*2];
    double fitness = mRetrievedFitnesses[_slave*2 + 1];
    for(uint i = 0; i < mUpdateList[_slave].size(); ++i){
        for(map<uint, Neuron*>::iterator iter = mUpdateList[_slave][i].begin(); iter != mUpdateList[_slave][i].end(); ++iter){
            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
        }
    }
    mUpdateList.erase(_slave);

    //update stagnation counter and set best solution
    if(realFitness <= mParameters.fitnessEpsilonThreshold || mBestFitness > fitness || mBestFitness == -1){
        mBestFitness = fitness;
        mBestRealFitness = realFitness;
        mBestSolution = *mSavedSolutions[_slave];
        _improved = true;
    }
}

void ESP::evaluateCompetitiveFitness(SimulationContainer* _simulationContainer){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    
    //setup initial work for slaves
    for(uint k = 0; k < mTotalRequests; ++k){
        //construct solution
        createNeuralNetworkPrimitives(neuralNetPrimitives);
        Solution *solution = constructSolution(neuralNetPrimitives);

        //save solution to map
        mSavedSolutions[k] = solution;

        //save neurons used so can update fitness later
        saveUpdateVec(neuralNetPrimitives, k);

        //assigns work to corresponding slave
        sendCompData(*solution, k);
    }

    //create solution and send to free slaves until cant create anymore, then wait for slaves to complete work
    while(createNeuralNetworkPrimitives(neuralNetPrimitives)){
        Solution *solution = constructSolution(neuralNetPrimitives);

        bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                //checks if slave thread
                if(k == 0 && mWorkStatus == NOWORK)
                    assigned = true;
                else if(k > 0){
                    MPI_Status status;
                    int received, receivedTeam;

                    MPI_Test(&mRequests[k], &received, &status);
                    MPI_Test(&mTeamRequests[k], &receivedTeam, &status);

                    if(received && receivedTeam)
                        assigned = true;
                }

                if(assigned){
                    updateCompFitness(k);

                    saveUpdateVec(neuralNetPrimitives, k);

                    delete mSavedSolutions[k];
                    mSavedSolutions.erase(k);
                    mSavedSolutions[k] = solution;

                    sendCompData(*solution, k);

                    break;
                }
            }
        }
    }

    //finish receiving work
    for(uint k = 0; k < mTotalRequests; ++k){
        bool completed = false;
        //poll the slave until it is complete
        while(!completed){
            Sleep(10);
            if(k == 0 && mWorkStatus == NOWORK)
                completed = true;
            else if(k > 0){
                MPI_Status status;
                int received;

                MPI_Test(&mRequests[k], &received, &status);

                if(received)
                    completed = true;
            }

            if(completed)
                updateCompFitness(k);
        }
        delete mSavedSolutions[k];
        mSavedSolutions.erase(k);
    }
}

void ESP::evaluateFitness(SimulationContainer* _simulationContainer){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    bool improved = false;

    //setup initial work for slaves
    for(uint k = 0; k < mTotalRequests; ++k){
        //construct solution
        createNeuralNetworkPrimitives(neuralNetPrimitives);
        Solution *solution = constructSolution(neuralNetPrimitives);

        //save solution to map
        mSavedSolutions[k] = solution;

        //saves neurons needed to be updated
        saveUpdateVec(neuralNetPrimitives, k);

        //send work to slaves
        sendData(*solution, k);
    }

    //create solution and send to free slaves until cant create anymore, then wait for slaves to complete work
    while(createNeuralNetworkPrimitives(neuralNetPrimitives)){
        Solution *solution = constructSolution(neuralNetPrimitives);

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
                    updateFitness(k, improved);

                    saveUpdateVec(neuralNetPrimitives, k);

                    delete mSavedSolutions[k];
                    mSavedSolutions.erase(k);
                    mSavedSolutions[k] = solution;

                    sendData(*solution, k);
                    break;
                }
            }
        }
    }

    //finish receiving work
    for(uint k = 0; k < mTotalRequests; ++k){
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

            if(completed)
                updateFitness(k, improved);
        }
        delete mSavedSolutions[k];
        mSavedSolutions.erase(k);
    }
}

bool ESP::createNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> out;
    bool completed = false;

    for(uint k = 0; k < mSubpopulations.size(); ++k){
        map<uint, Neuron*> output;
        map<uint, Neuron*> neuronCache;

		int teamID = -1;
		for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
			if(iter->second.second != 0){
				teamID = iter->second.first->getTeamID();
				break;
			}
		}

        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            if(iter->second.second == 0)
                neuronCache[iter->first] = new LeafNeuron(NULL, vector<double>(), teamID);
            else{
                Chromosome* chrom = iter->second.first->getUnevaluatedChromosome();
                if(chrom == NULL){
                    completed = true;
                    break;
                }

                neuronCache[iter->first] = dynamic_cast<ESPChromosome*>(chrom)->getNeuron();
                if(iter->second.second == 2)
                    output[iter->first] = neuronCache[iter->first];
            }
        }
        if(completed){
            for(map<uint, Neuron*>::iterator iter = neuronCache.begin(); iter != neuronCache.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
            }
            return false;
        }   
        out.push_back(make_pair(neuronCache, output));
    }
    _output = out;

    return true;
}

void ESP::setupSubpopulationStructure(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return;
    }

    pugi::xml_node root = doc.first_child();

    set<uint> teamIDs;
    for(pugi::xml_node currNetwork = root.first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
        map<uint, pair<ESPSubPopulation*, uint>> currNetworkSubpopulations;
        uint teamID = 0;
        if(currNetwork.attribute("TeamID").empty()){
            cerr << "Error: node does not have Team ID" << endl;
            //return;
        }
        else teamID = atoi(currNetwork.attribute("TeamID").value());
        teamIDs.insert(teamID);

        for(pugi::xml_node node = currNetwork.first_child(); node; node = node.next_sibling()){
            if(node.attribute("ID").empty()){
                cerr << "Error: node does not have ID" << endl;
                return;
            }
            
            uint neuronID = atoi(node.attribute("ID").value());
            
            if(node.attribute("Type").empty()){
                cerr << "Error: node does not have a Type" << endl;
                return;
            }

            uint neuronType;
            
            if(strcmp(node.attribute("Type").value(), "Input") == 0)
                neuronType = 0;
            else if(strcmp(node.attribute("Type").value(), "Hidden") == 0)
                neuronType = 1;
            else if(strcmp(node.attribute("Type").value(), "Output") == 0)
                neuronType = 2;

            ESPSubPopulation* subpop = neuronType == 0 ? NULL : new ESPSubPopulation(mParameters, &node, teamID);
            currNetworkSubpopulations[neuronID] = make_pair(subpop, neuronType);
        }

        mSubpopulations.push_back(currNetworkSubpopulations);
    }

    mNumTeams = teamIDs.size();
}

void ESP::runDeltaCodes(){
    for(uint i = 0; i < mSubpopulations.size(); ++i){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
            if(iter->second.second != 0)
                iter->second.first->generateDeltaCodes(mParameters.deltaCodeRadius);
        }
    }
}