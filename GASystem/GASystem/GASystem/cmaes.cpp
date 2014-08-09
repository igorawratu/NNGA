#include "cmaes.h"

using namespace std;

CMAES::CMAES(CMAESParameters _parameters){
    mParameters = _parameters;

    MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    assert(mTotalSlaveProcs > 0);

    int totalWork = _parameters.evalsPerCompChrom * _parameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests];

    mWorkStatus = NOWORK;
}
    
CMAES::~CMAES(){
    if(mUpdateList)
        delete [] mUpdateList;

    if(mRequests)
        delete [] mRequests;

    if(mRetrievedFitnesses)
        delete [] mRetrievedFitnesses;

    if(mRetrievedCompetitiveFitnesses)
        delete [] mRetrievedCompetitiveFitnesses;

    if(mRetrievedTeamIDs)
        delete [] mRetrievedTeamIDs;

    if(mTeamRequests)
        delete [] mTeamRequests;

    mSimulationContainer = 0;
    
    for(uint k = 0; k < mPopulation.size(); ++k)
        delete mPopulation[k];

    for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
        for(uint k = 0; k < iter->second.size(); ++k){
            delete iter->second[k];
        }
    }

}

Solution CMAES::train(SimulationContainer* _simulationContainer, string _outputFileName){
    if(!setup())
        return Solution();

    mSimulationContainer = _simulationContainer;
    
    //launch host worker thread
    boost::thread workerThread(boost::bind(&CMAES::hostwork, this));

    if(mNumTeams == 1 || mParameters.maxCompGenerations == 0)
        mStages = 1;
    else mStages = 2;

    for(mStage = 1; mStage <= mStages; ++mStage){ 
        if(mStage == 2){
            runDeltaCodes();
        }

        uint gen;

        if(mStages == 1 || mStage == 2){
            evaluateFitness(mPopulation);
            gen = mParameters.maxGenerations;
        }
        else{
            evaluateCompetitiveFitness();
            gen = mParameters.maxCompGenerations;
        }

        for(uint k = 0; k < gen; ++k){
            time_t t = time(0);
            
            cout << "Generation " << k << endl;
            //generate offspring

            //evaluate offspring
            cout << "Evaluating Fitness" << endl;
            if(mStages == 1 || mStage == 2)
                evaluateFitness(mPopulation);
            else evaluateCompetitiveFitness();

            //selection and recombination
            
            //step size control

            //covariance matrix update
            
            cout << "Time taken for this generation : " << time(0) - t << endl;
            
            //checks
            if(mStages == 1 || mStage == 2){
                if(mPopulation[0]->realFitness() <= mParameters.fitnessEpsilonThreshold){
                    Solution finalSolution(dynamic_cast<NNChromosome*>(mPopulation[0])->getNeuralNets());
                    finalSolution.fitness() = mPopulation[0]->fitness();

			        mWorkStatus = COMPLETE;
			        stopSlaves();
			        workerThread.join();

                    return finalSolution;
                }
            }
        }
    }

    Solution finalSolution(dynamic_cast<NNChromosome*>(mPopulation[0])->getNeuralNets());
    finalSolution.fitness() = mPopulation[0]->fitness();

    mWorkStatus = COMPLETE;
    stopSlaves();
    workerThread.join();

    return finalSolution;
}

bool CMAES::setup(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return false;
    }

    pugi::xml_node root = doc.first_child();

    set<int> teamIDs;
    for(pugi::xml_node currNetwork = root.first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
        uint teamID = 0;
        if(currNetwork.attribute("TeamID").empty()){
            cerr << "Error: node does not have Team ID" << endl;
        }
        else teamID = atoi(currNetwork.attribute("TeamID").value());
        teamIDs.insert(teamID);
    }

    mNumTeams = teamIDs.size();
    if(mNumTeams == 1){
	    mUpdateList = new int[mTotalRequests];
        
        for(uint k = 0; k < mParameters.populationSize; k++){
            NNChromosome* currChrom = new NNChromosome();
            if(!currChrom->initialize(&root)){
                cerr << "Error: unable to create chromosome in GA" << endl;
                return false;
            }
            else mPopulation.push_back(currChrom);
        }
    }
    else{
        mUpdateList = new int[mNumTeams * 2 * mTotalRequests];
        mTeamRequests = new MPI_Request[mTotalRequests];
        mRetrievedCompetitiveFitnesses = new double[mTotalRequests * mNumTeams];
        mRetrievedTeamIDs = new int[mTotalRequests * mNumTeams];

        for(set<int>::iterator iter = teamIDs.begin(); iter != teamIDs.end(); ++iter){
            mCompetitivePopulations[*iter] = vector<Chromosome*>();
        }

        for(uint k = 0; k < mParameters.populationSize; ++k){
            map<int, vector<NeuralNetwork>> nets;
            for(set<int>::iterator iter = teamIDs.begin(); iter != teamIDs.end(); ++iter){
                nets[*iter] = vector<NeuralNetwork>();
            }

            for(pugi::xml_node currNetwork = root.first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
                uint teamID = 0;
                if(currNetwork.attribute("TeamID").empty()){
                    cerr << "Error: node does not have Team ID" << endl;
                }
                else teamID = atoi(currNetwork.attribute("TeamID").value());

                NeuralNetwork currNN;
                if(!currNN.initialize(&currNetwork, true))
                    return false;
                nets[teamID].push_back(currNN);
            }

            for(map<int, vector<NeuralNetwork>>::iterator iter = nets.begin(); iter != nets.end(); ++iter){
                mCompetitivePopulations[iter->first].push_back(new NNChromosome(iter->second));
            }
        }
    }
}

void CMAES::evaluateFitness(vector<Chromosome*>& _population){
    uint currPos = 0;

	for(currPos = 0; currPos < mTotalRequests; ++currPos){
		Solution* currSolution = new Solution(dynamic_cast<NNChromosome*>(_population[currPos])->getNeuralNets());

		sendData(currSolution, currPos);

		mUpdateList[currPos] = currPos;
	}

	while(currPos < _population.size()){
		Solution* currSolution = new Solution(dynamic_cast<NNChromosome*>(_population[currPos])->getNeuralNets());
		bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                if(k == 0 && mWorkStatus == NOWORK){
                    assigned = true;
                }
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
			}
        }
    }
}

void CMAES::evaluateCompetitiveFitness(){
    //build tracklist
    map<int, vector<pair<uint, uint>>> competitiveTrackList;
    vector<int> teams;
    vector<uint> positions;
    Solution* currSolution;

    for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
        competitiveTrackList[iter->first] = vector<pair<uint, uint>>();
        for(uint k = 0; k < iter->second.size(); ++k)
            competitiveTrackList[iter->first].push_back(make_pair(k, 0));
    }

    //send initial messages
	for(uint k = 0; k < mTotalRequests; ++k){
		currSolution = constructCompSolution(competitiveTrackList, teams, positions);

		sendCompData(currSolution, k);

        int currUpdatePos = k * 2 * mNumTeams;
        for(uint i = 0; i < teams.size(); ++i){
		    mUpdateList[currUpdatePos++] = teams[i];
            mUpdateList[currUpdatePos++] = positions[i];
        }
        teams.clear();
        positions.clear();
	}

	while((currSolution = constructCompSolution(competitiveTrackList, teams, positions)) != NULL){
		bool assigned = false;

        //poll slaves
        while(!assigned){
            Sleep(10);
            for(uint k = 0; k < mTotalRequests; ++k){
                //check for slave completion
                if(k == 0 && mWorkStatus == NOWORK){
                    assigned = true;
                }
                else if(k > 0){
                    MPI_Status status;
                    int received, receivedTeam;

                    MPI_Test(&mRequests[k], &received, &status);
                    MPI_Test(&mTeamRequests[k], &receivedTeam, &status);

                    if(received && receivedTeam)
                        assigned = true;
                }

                if(assigned){
                    //update fitnesses
                    int startRetBuffer = k * mNumTeams;
                    int startUpdateList = k * mNumTeams * 2;
                    for(uint i = 0; i < mNumTeams; ++i){
                        for(uint l = 0; l < mNumTeams * 2; l+= 2){
                            if(mUpdateList[startUpdateList + l] == mRetrievedTeamIDs[startRetBuffer + i]){
                                mCompetitivePopulations[mUpdateList[startUpdateList + l]][mUpdateList[startUpdateList + l + 1]]->fitness() = mRetrievedCompetitiveFitnesses[startRetBuffer + i];
                                mCompetitivePopulations[mUpdateList[startUpdateList + l]][mUpdateList[startUpdateList + l + 1]]->realFitness() = mRetrievedCompetitiveFitnesses[startRetBuffer + i];
                            }
                        }
                    }

                    //updates updatelist and then sends data
                    for(uint i = 0; i < teams.size(); ++i){
		                mUpdateList[startUpdateList++] = teams[i];
                        mUpdateList[startUpdateList++] = positions[i];
                    }

                    sendCompData(currSolution, k);

                    break;
                }
            }
        }

        teams.clear();
        positions.clear();
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

            //update fitnesses
			if(completed){
                int startRetBuffer = k * mNumTeams;
                int startUpdateList = k * mNumTeams * 2;
                for(uint i = 0; i < mNumTeams; ++i){
                    for(uint l = 0; l < mNumTeams * 2; l+= 2){
                        if(mUpdateList[startUpdateList + l] == mRetrievedTeamIDs[startRetBuffer + i]){
                            mCompetitivePopulations[mUpdateList[startUpdateList + l]][mUpdateList[startUpdateList + l + 1]]->fitness() = mRetrievedCompetitiveFitnesses[startRetBuffer + i];
                            mCompetitivePopulations[mUpdateList[startUpdateList + l]][mUpdateList[startUpdateList + l + 1]]->realFitness() = mRetrievedCompetitiveFitnesses[startRetBuffer + i];
                        }
                    }
                }
			}
        }
    }
}

void CMAES::runDeltaCodes(){
    //cleanup population if it exists(it shouldnt)
    for(uint k = 0; k < mPopulation.size(); ++k){
        delete mPopulation[k];
    }
    mPopulation.clear();

    vector<NeuralNetwork> solutionNets;

    //get best performers in each team and append to solutionNets
    for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
        vector<NeuralNetwork> currTeamNets = dynamic_cast<NNChromosome*>(mCompetitivePopulations[iter->first][0])->getNeuralNets();
        solutionNets.insert(solutionNets.end(), currTeamNets.begin(), currTeamNets.end());
    }

    Chromosome* seedChrom = new NNChromosome(solutionNets);

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(-mParameters.deltaCodeRadius, mParameters.deltaCodeRadius);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    mPopulation.push_back(seedChrom);

    while(mPopulation.size() < mParameters.populationSize){
        vector<map<uint, vector<double>>> newWeights = seedChrom->getWeightData();
        for(uint k = 0; k < newWeights.size(); ++k){
            for(map<uint, vector<double>>::iterator iter = newWeights[k].begin(); iter != newWeights[k].end(); ++iter){
                for(uint i = 0; i < iter->second.size(); ++i){
                    iter->second[i] = gen();
                }
            }
        }

        Chromosome* newChrom = seedChrom->clone();
        newChrom->setWeights(newWeights);
        mPopulation.push_back(newChrom);
    }
}

void CMAES::stopSlaves(){
    for(uint k = 1; k < mTotalSlaveProcs; ++k){
        int stopmsg[3] = {-1, -1, -1};
        MPI_Send(&stopmsg[0], 3, MPI_INT, k, 1, MPI_COMM_WORLD);
    }
}

void CMAES::hostwork(){
    while(mWorkStatus != COMPLETE){
        if(mWorkStatus == WORK){
            mSimulationContainer->runFullSimulation(mSavedSlaveSolution);
            mSimulationContainer->resetSimulation();

            if(mStages == 1 || mStage == 2){
                mRetrievedFitnesses[0] = mSavedSlaveSolution->realFitness();
                mRetrievedFitnesses[1] = mSavedSlaveSolution->fitness();
            }
            else{
                vector<CompetitiveFitness> compFitnesses = mSavedSlaveSolution->competitiveFitness();
                for(uint k = 0; k < compFitnesses.size(); ++k){
                    mRetrievedTeamIDs[k] = compFitnesses[k].first;
                    mRetrievedCompetitiveFitnesses[k] = compFitnesses[k].second;
                }
            }

            delete mSavedSlaveSolution;
            mSavedSlaveSolution = 0;
            mWorkStatus = NOWORK;
        }
    }
}

void CMAES::sendCompData(Solution* _solution, int _slave){
    if(_slave == 0){
        mSavedSlaveSolution = _solution;
        mWorkStatus = WORK;
    }
    else{
        //construct serialized data to send through
        int initialDat[4];
        int *nodes, *format;
        double* weights;

        _solution->serialize(nodes, format, weights, initialDat[0], initialDat[1], initialDat[2]);
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
        delete _solution;
    }
}

void CMAES::sendData(Solution* _solution, int _slave){
    if(_slave == 0){
        mSavedSlaveSolution = _solution;
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

Solution* CMAES::constructCompSolution(map<int, vector<pair<uint, uint>>>& _competitiveTracklist, vector<int>& _team, vector<uint>& _position){
    vector<NeuralNetwork> solutionNets;

    //loop through different teams
    for(map<int, vector<pair<uint, uint>>>::iterator iter = _competitiveTracklist.begin(); iter != _competitiveTracklist.end(); ++iter){
        int pos;

        //acquire position of neural nets
        if(iter->second.size() == 0)
            break;
        else if(iter->second.size() == 1)
            pos = 0;
        else{
            boost::mt19937 rng(rand());
            boost::uniform_int<> dist(0, iter->second.size() - 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

            pos = gen();
        }

        //append networks obtained to back of solution networks
        vector<NeuralNetwork> currTeamNets = dynamic_cast<NNChromosome*>(mCompetitivePopulations[iter->first][iter->second[pos].first])->getNeuralNets();
        solutionNets.insert(solutionNets.end(), currTeamNets.begin(), currTeamNets.end());
        _team.push_back(iter->first);
        _position.push_back(iter->second[pos].first);

        //remove possibility of sampling chosen chromosome again if it has been evaluated a prespecified amount of times already
        iter->second[pos].second++;
        if(iter->second[pos].second == mParameters.evalsPerCompChrom){
            iter->second.erase(iter->second.begin() + pos);
        }
    }

    if(solutionNets.size() == 0)
        return NULL;
    else return new Solution(solutionNets);
}