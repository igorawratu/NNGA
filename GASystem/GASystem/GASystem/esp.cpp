#include "esp.h"

ESP::ESP(ESPParameters _parameters){
    mParameters = _parameters;

    setupSubpopulationStructure();

    MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    assert(mTotalSlaveProcs > 0);

    int totalWork = _parameters.sampleEvaluationsPerChromosome * _parameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests * 2];

    if(mNumTeams > 1){
        mRetrievedCompetitiveFitnesses = new double[mTotalRequests * mNumTeams];
        mRetrievedTeamIDs = new double[mTotalRequests * mNumTeams];
    }
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

    mStages = mNumTeams == 1? 1 : 2;

    for(mStage = 1; mStage <= mStages; ++mStage){ 
        if(mStage == 2)
            runDeltaCodes(_simulationContainer);

        if(mStages == 1 || mStage == 2)
            evaluateFitness(_simulationContainer);
        else evaluateCompetitiveFitness(_simulationContainer);

        for(uint k = 0; k < mParameters.maxGenerations; ++k){
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
                    if(iter->second.second != 0)
                        iter->second.first->nextGeneration();
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
                    mRetrievedTeamIDs[k] = compFitnesses[k].get<0>();
                    mRetrievedCompetitiveFitnesses[k] = compFitnesses[k].get<1>();
                }
            }

            mWorkStatus = NOWORK;
        }
    }
}

void evaluateCompetitiveFitness(SimulationContainer* _simulationContainer){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    
    //setup initial work for slaves
    for(uint k = 0; k < mTotalRequests; ++k){
        //construct solution
        createNeuralNetworkPrimitives(neuralNetPrimitives);
        vector<NeuralNetwork> neuralNets;

        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[i].setStructure(neuralNetPrimitives[i].first, neuralNetPrimitives[i].second);
        }

        Solution *solution = new Solution(neuralNets);

        //save solution to map
        mSavedSolutions[k] = solution;

        //saves neurons needed to be updated
        vector<map<uint, Neuron*>> currUpdateVec;
            
        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
            currUpdateVec.push_back(map<uint, Neuron*>());
            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
                else currUpdateVec[i][iter->first] = iter->second;
            }
        }

        mUpdateList[k] = currUpdateVec;

        if(k == 0)
            mWorkStatus = WORK;
        else{
            //construct serialized data to send through
            int initialDat[3];
            int *nodes, *format;
            double* weights;

            solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);

            MPI_Send(&initialDat[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(nodes, initialDat[0], MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(format, initialDat[1], MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(weights, initialDat[2], MPI_DOUBLE, k, 1, MPI_COMM_WORLD);

            //setup corresponding nonblocking receive call
            MPI_Irecv(&mRetrievedCompetitiveFitnesses[k * mNumTeams], mNumTeams, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);
            MPI_Irecv(&mRetrievedTeamIDs[k * mNumTeams], mNumTeams, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);

            delete [] nodes;
            delete [] format;
            delete [] weights;
        }
    }

    //create solution and send to free slaves until cant create anymore, then wait for slaves to complete work
    while(createNeuralNetworkPrimitives(neuralNetPrimitives)){
        //create solution
        vector<NeuralNetwork> neuralNets;
        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[k].setStructure(neuralNetPrimitives[k].first, neuralNetPrimitives[k].second);
        }
        Solution *solution = new Solution(neuralNets);

        bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                //checks if slave thread
                if(k == 0){
                    if(mWorkStatus == NOWORK){
                        assigned = true;

                        //update chromosomes with fitness
                        for(uint i = 0; i < mUpdateList[k].size(); ++i){
                            for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                                uint currFitnessTeamID = mSubpopulations[i][iter->first].first->getTeamID();
                                double fitness = 0;
                                for(uint i = k; i < k + mNumTeams; ++i){
                                    if(mTeamIDs[i] == currFitnessTeamID){
                                        fitness = mRetrievedCompetitiveFitnesses[i];
                                        break;
                                    }
                                }
                                mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, fitness);
                            }
                        }
                        mUpdateList.erase(k);

                        //add chromosomes to corresponding map position
                        vector<map<uint, Neuron*>> currUpdateVec;
                        
                        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
                            currUpdateVec.push_back(map<uint, Neuron*>());
                            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                                if(iter->second->getNeuronType() == LEAF)
                                    delete iter->second;
                                else currUpdateVec[i][iter->first] = iter->second;
                            }
                        }

                        mUpdateList[k] = currUpdateVec;

                        delete mSavedSolutions[k];
                        mSavedSolutions.erase(k);
                        mSavedSolutions[k] = solution;

                        mWorkStatus = WORK;

                        break;
                    }
                }
                else{
                    MPI_Status status;
                    int received;

                    MPI_Test(&mRequests[k], &received, &status);

                    if(received){
                        //construct serialized data to send through
                        int initialDat[3];
                        int *nodes, *format;
                        double* weights;

                        solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);

                        assigned = true;

                        //update chromosomes with fitness
                        for(uint i = 0; i < mUpdateList[k].size(); ++i){
                            for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                                uint currFitnessTeamID = mSubpopulations[i][iter->first].first->getTeamID();
                                double fitness = 0;
                                for(uint i = k; i < k + mNumTeams; ++i){
                                    if(mTeamIDs[i] == currFitnessTeamID){
                                        fitness = mRetrievedCompetitiveFitnesses[i];
                                        break;
                                    }
                                }
                                mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, fitness);
                            }
                        }
                        mUpdateList.erase(k);

                        //assign new work
                        MPI_Send(&initialDat[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(nodes, initialDat[0], MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(format, initialDat[1], MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(weights, initialDat[2], MPI_DOUBLE, k, 1, MPI_COMM_WORLD);

                        //setup corresponding nonblocking receive call
                        MPI_Irecv(&mRetrievedCompetitiveFitnesses[k * mNumTeams], mNumTeams, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);
                        MPI_Irecv(&mRetrievedTeamIDs[k * mNumTeams], mNumTeams, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);

                        //add chromosomes to corresponding map position
                        vector<map<uint, Neuron*>> currUpdateVec;
                        
                        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
                            currUpdateVec.push_back(map<uint, Neuron*>());
                            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                                if(iter->second->getNeuronType() == LEAF)
                                    delete iter->second;
                                else currUpdateVec[i][iter->first] = iter->second;
                            }
                        }

                        mUpdateList[k] = currUpdateVec;

                        delete mSavedSolutions[k];
                        mSavedSolutions.erase(k);
                        mSavedSolutions[k] = solution;

                        delete [] nodes;
                        delete [] format;
                        delete [] weights;

                        //stop looping
                        break;
                    }
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
            if(k == 0){
                if(mWorkStatus == NOWORK){
                    double realFitness = mRetrievedFitnesses[k*2];
                    double fitness = mRetrievedFitnesses[k*2 + 1];
                    for(uint i = 0; i < mUpdateList[k].size(); ++i){
                        for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
                        }
                    }
                    mUpdateList.erase(k);
                    completed = true;
                }
            }
            else{
                //sleep
                MPI_Status status;
                int received;

                MPI_Test(&mRequests[k], &received, &status);

                if(received){
                    //update chromosomes with fitness
                    for(uint i = 0; i < mUpdateList[k].size(); ++i){
                        for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                            uint currFitnessTeamID = mSubpopulations[i][iter->first].first->getTeamID();
                            double fitness = 0;
                            for(uint i = k; i < k + mNumTeams; ++i){
                                if(mTeamIDs[i] == currFitnessTeamID){
                                    fitness = mRetrievedCompetitiveFitnesses[i];
                                    break;
                                }
                            }
                            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, fitness);
                        }
                    }
                    mUpdateList.erase(k);

                    completed = true;
                }
            }
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
        vector<NeuralNetwork> neuralNets;

        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[i].setStructure(neuralNetPrimitives[i].first, neuralNetPrimitives[i].second);
        }

        Solution *solution = new Solution(neuralNets);

        //save solution to map
        mSavedSolutions[k] = solution;

        //saves neurons needed to be updated
        vector<map<uint, Neuron*>> currUpdateVec;
            
        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
            currUpdateVec.push_back(map<uint, Neuron*>());
            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
                else currUpdateVec[i][iter->first] = iter->second;
            }
        }

        mUpdateList[k] = currUpdateVec;

        if(k == 0)
            mWorkStatus = WORK;
        else{
            //construct serialized data to send through
            int initialDat[3];
            int *nodes, *format;
            double* weights;

            solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);

            MPI_Send(&initialDat[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(nodes, initialDat[0], MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(format, initialDat[1], MPI_INT, k, 1, MPI_COMM_WORLD);
            MPI_Send(weights, initialDat[2], MPI_DOUBLE, k, 1, MPI_COMM_WORLD);

            //setup corresponding nonblocking receive call
            MPI_Irecv(&mRetrievedFitnesses[k * 2], 2, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);

            delete [] nodes;
            delete [] format;
            delete [] weights;
        }
    }

    //create solution and send to free slaves until cant create anymore, then wait for slaves to complete work
    while(createNeuralNetworkPrimitives(neuralNetPrimitives)){
        //create solution
        vector<NeuralNetwork> neuralNets;
        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[k].setStructure(neuralNetPrimitives[k].first, neuralNetPrimitives[k].second);
        }
        Solution *solution = new Solution(neuralNets);

        bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                if(k == 0){
                    if(mWorkStatus == NOWORK){
                        assigned = true;

                        //update chromosomes with fitness
                        double realFitness = mRetrievedFitnesses[k*2];
                        double fitness = mRetrievedFitnesses[k*2 + 1];
                        for(uint i = 0; i < mUpdateList[k].size(); ++i){
                            for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                                mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
                            }
                        }
                        mUpdateList.erase(k);

                        //update stagnation counter and set best solution
                        if(realFitness <= mParameters.fitnessEpsilonThreshold || mBestFitness > fitness || mBestFitness == -1){
                            mBestFitness = fitness;
                            mBestRealFitness = realFitness;
                            mBestSolution = *mSavedSolutions[k];
                            improved = true;
                        }

                        //add chromosomes to corresponding map position
                        vector<map<uint, Neuron*>> currUpdateVec;
                        
                        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
                            currUpdateVec.push_back(map<uint, Neuron*>());
                            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                                if(iter->second->getNeuronType() == LEAF)
                                    delete iter->second;
                                else currUpdateVec[i][iter->first] = iter->second;
                            }
                        }

                        mUpdateList[k] = currUpdateVec;

                        delete mSavedSolutions[k];
                        mSavedSolutions.erase(k);
                        mSavedSolutions[k] = solution;

                        mWorkStatus = WORK;

                        break;
                    }
                }
                else{
                    MPI_Status status;
                    int received;

                    MPI_Test(&mRequests[k], &received, &status);

                    if(received){
                        //construct serialized data to send through
                        int initialDat[3];
                        int *nodes, *format;
                        double* weights;

                        solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);

                        assigned = true;

                        //update chromosomes with fitness
                        double realFitness = mRetrievedFitnesses[k*2];
                        double fitness = mRetrievedFitnesses[k*2 + 1];
                        for(uint i = 0; i < mUpdateList[k].size(); ++i){
                            for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                                mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
                            }
                        }
                        mUpdateList.erase(k);

                        //update stagnation counter and set best solution
                        if(realFitness <= mParameters.fitnessEpsilonThreshold || mBestFitness > fitness || mBestFitness == -1){
                            mBestFitness = fitness;
                            mBestRealFitness = realFitness;
                            mBestSolution = *mSavedSolutions[k];
                            improved = true;
                        }

                        //assign new work
                        MPI_Send(&initialDat[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(nodes, initialDat[0], MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(format, initialDat[1], MPI_INT, k, 1, MPI_COMM_WORLD);
                        MPI_Send(weights, initialDat[2], MPI_DOUBLE, k, 1, MPI_COMM_WORLD);

                        //setup corresponding nonblocking receive call
                        MPI_Irecv(&mRetrievedFitnesses[k * 2], 2, MPI_DOUBLE, k, 1, MPI_COMM_WORLD, &mRequests[k]);

                        //add chromosomes to corresponding map position
                        vector<map<uint, Neuron*>> currUpdateVec;
                        
                        for(uint i = 0; i < neuralNetPrimitives.size(); ++i){
                            currUpdateVec.push_back(map<uint, Neuron*>());
                            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[i].first.begin(); iter != neuralNetPrimitives[i].first.end(); ++iter){
                                if(iter->second->getNeuronType() == LEAF)
                                    delete iter->second;
                                else currUpdateVec[i][iter->first] = iter->second;
                            }
                        }

                        mUpdateList[k] = currUpdateVec;

                        delete mSavedSolutions[k];
                        mSavedSolutions.erase(k);
                        mSavedSolutions[k] = solution;

                        delete [] nodes;
                        delete [] format;
                        delete [] weights;

                        //stop looping
                        break;
                    }
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
            if(k == 0){
                if(mWorkStatus == NOWORK){
                    double realFitness = mRetrievedFitnesses[k*2];
                    double fitness = mRetrievedFitnesses[k*2 + 1];
                    for(uint i = 0; i < mUpdateList[k].size(); ++i){
                        for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
                        }
                    }
                    mUpdateList.erase(k);

                    //update stagnation counter and set best solution
                    if(realFitness <= mParameters.fitnessEpsilonThreshold || mBestFitness > fitness || mBestFitness == -1){
                        mBestFitness = fitness;
                        mBestRealFitness = realFitness;
                        mBestSolution = *mSavedSolutions[k];
                        improved = true;
                    }

                    completed = true;
                }
            }
            else{
                //sleep
                MPI_Status status;
                int received;

                MPI_Test(&mRequests[k], &received, &status);

                if(received){
                    completed = true;
                    //update chromosomes with fitness
                    double realFitness = mRetrievedFitnesses[k*2];
                    double fitness = mRetrievedFitnesses[k*2 + 1];
                    for(uint i = 0; i < mUpdateList[k].size(); ++i){
                        for(map<uint, Neuron*>::iterator iter = mUpdateList[k][i].begin(); iter != mUpdateList[k][i].end(); ++iter){
                            mSubpopulations[i][iter->first].first->setChromosomeFitness(iter->second, fitness, realFitness);
                        }
                    }
                    mUpdateList.erase(k);

                    //update stagnation counter and set best solution
                    if(realFitness <= mParameters.fitnessEpsilonThreshold || mBestFitness > fitness || mBestFitness == -1){
                        mBestFitness = fitness;
                        mBestRealFitness = realFitness;
                        mBestSolution = *mSavedSolutions[k];
                        improved = true;
                    }
                }
            }
        }
        delete mSavedSolutions[k];
        mSavedSolutions.erase(k);
    }

    mStagnationCounter = improved ? 0 : mStagnationCounter + 1;
}

bool ESP::createNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> out;
    bool completed = false;

    for(uint k = 0; k < mSubpopulations.size(); ++k){
        map<uint, Neuron*> output;
        map<uint, Neuron*> neuronCache;

        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            if(iter->second.second == 0)
                neuronCache[iter->first] = new LeafNeuron(NULL, vector<double>());
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

bool ESP::createDeltaNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> out;
    bool completed = false;

    for(uint k = 0; k < mSubpopulations.size(); ++k){
        map<uint, Neuron*> output;
        map<uint, Neuron*> neuronCache;

        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            if(iter->second.second == 0)
                neuronCache[iter->first] = new LeafNeuron(NULL, vector<double>());
            else{
                Chromosome* chrom = iter->second.first->getUnevaluatedDeltaCode();
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
            cerr << "Error: node does not have ID" << endl;
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

            ESPSubPopulation* subpop = neuronType == 0 ? NULL : new ESPSubPopulation(mParameters, &node);
            subpop->setTeamID(teamID);
            currNetworkSubpopulations[neuronID] = make_pair(subpop, neuronType);
        }

        mSubpopulations.push_back(currNetworkSubpopulations);
    }

    mNumTeams = teamIDs.size();
}

void ESP::runDeltaCodes(SimulationContainer* _simulationContainer){
    for(uint i = 0; i < mSubpopulations.size(); ++i){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
            if(iter->second.second != 0)
                iter->second.first->generateDeltaCodes();
        }
    }

    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    while(createDeltaNeuralNetworkPrimitives(neuralNetPrimitives)){
        vector<NeuralNetwork> neuralNets;
        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[k].setStructure(neuralNetPrimitives[k].first, neuralNetPrimitives[k].second);
        }
        Solution solution(neuralNets);
        _simulationContainer->runFullSimulation(&solution);
        _simulationContainer->resetSimulation();

        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[k].first.begin(); iter != neuralNetPrimitives[k].first.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
                else{
                    mSubpopulations[k][iter->first].first->setDeltaCodeFitness(iter->second, solution.fitness(), solution.realFitness());
                }
            }
        }
    }

    for(uint i = 0; i < mSubpopulations.size(); ++i){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
            if(iter->second.second != 0)
                iter->second.first->integrateDeltaCodes();
        }
    }
}