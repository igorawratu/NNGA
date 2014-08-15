#include "cmaes.h"

using namespace std;

CMAES::CMAES(CMAESParameters _parameters){
    mParameters = _parameters;

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
    //setup ANN structure
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

        //non competitive stage
        if(mStages == 1 || mStage == 2){
            //obtain dimensions of problem
            int dims = calcDims(mPopulation[0]);

            //create mean with 0s initially
            Eigen::MatrixXd currMean(dims, 1);
            for(uint k = 0; k < dims; ++k)
                currMean(k, 0) = 0;

            //initialise evolution paths, step size, and covariance matrix
            Eigen::MatrixXd covMat(dims, dims);
            covMat = Eigen::MatrixXd::Identity(dims, dims);
            Eigen::MatrixXd pc(dims, 1);
            Eigen::MatrixXd psig(dims, 1);
            double stepSize = mParameters.initStepsize;

            //perform eigenDecomp on initial covariance matrix
            Eigen::MatrixXd eigenVectors(dims, dims);
            Eigen::MatrixXd eigenValuesSqrt(dims, 1);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covMat);
            eigenVectors = eigenSolver.eigenvectors();
            eigenValuesSqrt = eigenSolver.eigenvalues();

            double csig = (mParameters.mewEff + 2) / (dims + mParameters.mewEff + 5);

            double dsigTemp = sqrt((mParameters.mewEff - 1) / (dims + 1)) - 1;
            dsigTemp = dsigTemp > 0 ? dsigTemp : 0;
            double dsig = 1 + 2 * dsigTemp + csig;
            double cc = (4 + mParameters.mewEff / dims) / (dims + 4 + ((2*mParameters.mewEff) / dims));
            double c1 = 2 / (pow(dims + 1.3, 2) + mParameters.mewEff);
            
            double cmewTemp = 2 * ((mParameters.mewEff - 2 + 1/mParameters.mewEff)/(pow((double)dims + 2, 2) + (2*mParameters.mewEff)/2));
            double cmew = cmewTemp < (1 - c1) ? cmewTemp : (1 - c1);

            //important that eigenvalues are square rooted, for later use in algorithm as well as use in generating multivariate data
            for(int k = 0; k < dims; ++k) {
                eigenValuesSqrt(k, 0) = sqrt(eigenValuesSqrt(k, 0));
            }

            for(uint k = 0; k < dims; ++k){
                pc(k, 0) = 0;
                psig(k, 0) = 0;
            }

            //regenerates population so that it samples from a multivariate normal(this is a bit of a hack, the system originally was made so that initial population is sampled off a 
            //uniform distribution. DOES NOT get run if this is run after the competitive step, reason being we will lose delta coding effects)
            if(mStages == 1){
                generateOffspring(mPopulation, eigenVectors, eigenValuesSqrt, currMean, stepSize, dims);
            }

            //evaluate initial population
            evaluateFitness(mPopulation);

            for(uint k = 0; k < mParameters.maxGenerations; ++k){
                time_t t = time(0);
                
                cout << "Generation " << k << endl;

                //selection and recombination
                Eigen::MatrixXd newMean(dims, 1);

                vector<Eigen::MatrixXd> yilambdas;
                calcMean(mPopulation, newMean, dims, yilambdas, stepSize);

                //step size control
                Eigen::MatrixXd cNegHalf(dims, dims);
                Eigen::MatrixXd diagEigenValsSqrt(dims, dims);

                Eigen::MatrixXd yw(dims, 1);
                for(uint i = 0; i < dims; ++i){
                    yw(i, 0) = (newMean(i, 0) - currMean(i, 0)) / stepSize;
                }

                diagEigenValsSqrt = Eigen::MatrixXd::Zero(dims, dims);
                for(uint i = 0; i < dims; ++i){
                    diagEigenValsSqrt(i, i) = eigenValuesSqrt(i, 0);
                }
                cNegHalf = eigenVectors * diagEigenValsSqrt * eigenVectors.transpose();

                psig = (1 - csig) * psig + sqrt(csig * (2 - csig) * mParameters.mewEff) * cNegHalf * yw;
                
                double psmag = 0;
                for(uint k = 0; k < dims; ++k){
                    psmag += psig(k, 0) * psig(k, 0);
                }
                psmag = sqrt(psmag);

                double chiN = sqrt((double)dims) * (1 - 1/(4 * dims) + 1/(21 * dims * dims));
                double expTerm = (csig / dsig) * (psmag / chiN - 1);

                stepSize *= exp(expTerm);

                //covariance matrix update
                double hsigT1, hsigT2;
                hsigT1 = psmag / sqrt(1 - pow(1. - csig, 2. * (k + 1.)));
                hsigT2 = (1.4 + 2/(dims + 1)) * chiN;
                int hsig = hsigT1 < hsigT2 ? 1 : 0;

                pc = (1 - cc) * pc + hsig * sqrt(cc * (2 - cc) * mParameters.mewEff) * yw;
                
                Eigen::MatrixXd mem(dims, dims);
                mem = (1 - c1 - cmew) * covMat;

                Eigen::MatrixXd rankOne(dims, dims);
                double deltaHsig = (1 - hsig) * cc * (2 - cc);
                rankOne = c1 * (pc * pc.transpose() + deltaHsig * covMat);

                Eigen::MatrixXd rankMew(dims, dims);
                rankMew = Eigen::MatrixXd::Zero(dims, dims);

                for(uint i = 0; i < mParameters.parentSize; ++i){
                    rankMew += mParameters.weights(i, 0) * yilambdas[i] * yilambdas[i].transpose();
                }
                rankMew *= cmew;

                covMat = mem + rankOne + rankMew;

                //update the eigen decomposition of the covariance matrix
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covMat);
                eigenVectors = eigenSolver.eigenvectors();
                eigenValuesSqrt = eigenSolver.eigenvalues();

                for(int k = 0; k < dims; ++k) {
                    eigenValuesSqrt(k, 0) = sqrt(eigenValuesSqrt(k, 0));
                }
                
                //update mean
                currMean = newMean;

                //generate offspring
                cout << "Generating offspring" << endl;
                generateOffspring(mPopulation, eigenVectors, eigenValuesSqrt, currMean, stepSize, dims);

                //evaluate offspring
                cout << "Evaluating Fitness" << endl;
                evaluateFitness(mPopulation);

                cout << "Current population fitnesses..." << endl;
                for(uint i = 0; i < mPopulation.size(); i++)
                    cout << mPopulation[i]->fitness() << " " << mPopulation[i]->realFitness() << " | ";
                cout << endl;
                
                cout << "Time taken for this generation : " << time(0) - t << endl;
                
                //checks for termination
                if(mPopulation[0]->realFitness() <= mParameters.fitnessEpsilonThreshold){
                    cout << "Termination criteria met, fitness obtained below epsilon" << endl;

                    Solution finalSolution(dynamic_cast<NNChromosome*>(mPopulation[0])->getNeuralNets());
                    finalSolution.fitness() = mPopulation[0]->fitness();

		            mWorkStatus = COMPLETE;
		            stopSlaves();
		            workerThread.join();

                    return finalSolution;
                }
            }
        }
        //competitive stage
        else{
            map<int, Eigen::MatrixXd> covMats;
            map<int, Eigen::MatrixXd> pcs;
            map<int, Eigen::MatrixXd> psigs;
            map<int, double> stepSizes;
            map<int, Eigen::MatrixXd> currMeans;
            map<int, int> dims;
            map<int, Eigen::MatrixXd> eigenVectors;
            map<int, Eigen::MatrixXd> eigenValuesSqrt;
            map<int, double> csigs;
            map<int, double> dsigs;
            map<int, double> ccs;
            map<int, double> c1s;
            map<int, double> cmews;

            for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
                //obtain dimensions of problem
                dims[iter->first] = calcDims(iter->second[0]);

                //set constants
                csigs[iter->first] = (mParameters.mewEff + 2) / (dims[iter->first] + mParameters.mewEff + 5);

                double dsigTemp = sqrt((mParameters.mewEff - 1) / (dims[iter->first] + 1)) - 1;
                dsigTemp = dsigTemp > 0 ? dsigTemp : 0;
                dsigs[iter->first] = 1 + 2 * dsigTemp + csigs[iter->first];
                ccs[iter->first] = (4 + mParameters.mewEff / dims[iter->first]) / (dims[iter->first] + 4 + (2*mParameters.mewEff) / dims[iter->first]);
                c1s[iter->first] = 2 / (pow(dims[iter->first] + 1.3, 2) + mParameters.mewEff);
                
                double cmewTemp = 2 * ((mParameters.mewEff - 2 + 1/mParameters.mewEff)/(pow((double)dims[iter->first] + 2, 2) + (2*mParameters.mewEff)/2));
                cmews[iter->first] = cmewTemp < (1 - c1s[iter->first]) ? cmewTemp : (1 - c1s[iter->first]);

                //create mean with 0s initially
                currMeans[iter->first] = Eigen::MatrixXd(dims[iter->first], 1);
                for(uint k = 0; k < dims[iter->first]; ++k)
                    currMeans[iter->first](k, 0) = 0;
                
                //initialise evolution paths, step size, and covariance matrix
                covMats[iter->first] = Eigen::MatrixXd(dims[iter->first], dims[iter->first]);
                covMats[iter->first] = Eigen::MatrixXd::Identity(dims[iter->first], dims[iter->first]);
                pcs[iter->first] = Eigen::MatrixXd(dims[iter->first], 1);
                psigs[iter->first] = Eigen::MatrixXd(dims[iter->first], 1);
                stepSizes[iter->first] = mParameters.initStepsize;

                //perform eigenDecomp on initial covariance matrix
                eigenVectors[iter->first] = Eigen::MatrixXd(dims[iter->first], dims[iter->first]);
                eigenValuesSqrt[iter->first] = Eigen::MatrixXd(dims[iter->first], 1);
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covMats[iter->first]);
                eigenVectors[iter->first] = eigenSolver.eigenvectors();
                eigenValuesSqrt[iter->first] = eigenSolver.eigenvalues();

                //important that eigenvalues are square rooted, for later use in algorithm as well as use in generating multivariate data
                for(int k = 0; k < dims[iter->first]; ++k) {
                    eigenValuesSqrt[iter->first](k, 0) = sqrt(eigenValuesSqrt[iter->first](k, 0));
                }

                for(uint k = 0; k < dims[iter->first]; ++k){
                    pcs[iter->first](k, 0) = 0;
                    psigs[iter->first](k, 0) = 0;
                }

                //regenerates population so that it samples from a multivariate normal(this is a bit of a hack, the system originally was made so that initial population is sampled off a 
                //uniform distribution.)
                generateOffspring(iter->second, eigenVectors[iter->first], eigenValuesSqrt[iter->first], currMeans[iter->first], stepSizes[iter->first], dims[iter->first]);
            }

            //evaluate initial population
            evaluateCompetitiveFitness();

            for(uint k = 0; k < mParameters.maxCompGenerations; ++k){
                time_t t = time(0);
                cout << "Generation " << k << endl;

                for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
                    //selection and recombination
                    Eigen::MatrixXd newMean(dims[iter->first], 1);

                    
                    vector<Eigen::MatrixXd> yilambdas;
                    calcMean(iter->second, newMean, dims[iter->first], yilambdas, stepSizes[iter->first]);

                     //step size control
                    Eigen::MatrixXd cNegHalf(dims[iter->first], dims[iter->first]);
                    Eigen::MatrixXd diagEigenValsSqrt(dims[iter->first], dims[iter->first]);

                    Eigen::MatrixXd yw(dims[iter->first], 1);
                    for(uint i = 0; i < dims[iter->first]; ++i){
                        yw(i, 0) = (newMean(i, 0) - currMeans[iter->first](i, 0)) / stepSizes[iter->first];
                    }

                    diagEigenValsSqrt = Eigen::MatrixXd::Zero(dims[iter->first], dims[iter->first]);
                    for(uint i = 0; i < dims[iter->first]; ++i){
                        diagEigenValsSqrt(i, i) = eigenValuesSqrt[iter->first](i, 0);
                    }

                    cNegHalf = eigenVectors[iter->first] * diagEigenValsSqrt * eigenVectors[iter->first].transpose();

                    psigs[iter->first] = (1 - csigs[iter->first]) * psigs[iter->first] + sqrt(csigs[iter->first] * (2 - csigs[iter->first]) * mParameters.mewEff) * cNegHalf * yw;

                    double psmag = 0;
                    for(uint k = 0; k < dims[iter->first]; ++k){
                        psmag += psigs[iter->first](k, 0) * psigs[iter->first](k, 0);
                    }
                    psmag = sqrt(psmag);

                    double chiN = sqrt((double)dims[iter->first]) * (1 - 1/(4 * dims[iter->first]) + 1/(21 * dims[iter->first] * dims[iter->first]));
                    double expTerm = (csigs[iter->first] / dsigs[iter->first]) * (psmag / chiN - 1);

                    stepSizes[iter->first] *= exp(expTerm);

                     //covariance matrix update
                    double hsigT1, hsigT2;
                    hsigT1 = psmag / sqrt(1 - pow(1. - csigs[iter->first], 2. * (k + 1.)));
                    hsigT2 = (1.4 + 2/(dims[iter->first] + 1)) * chiN;
                    int hsig = hsigT1 < hsigT2 ? 1 : 0;

                    pcs[iter->first] = (1 - ccs[iter->first]) * pcs[iter->first] + hsig * sqrt(ccs[iter->first] * (2 - ccs[iter->first]) * mParameters.mewEff) * yw;
                
                    Eigen::MatrixXd mem(dims[iter->first], dims[iter->first]);
                    mem = (1 - c1s[iter->first] - cmews[iter->first]) * covMats[iter->first];

                    Eigen::MatrixXd rankOne(dims[iter->first], dims[iter->first]);
                    double deltaHsig = (1 - hsig) * ccs[iter->first] * (2 - ccs[iter->first]);
                    rankOne = c1s[iter->first] * (pcs[iter->first] * pcs[iter->first].transpose() + deltaHsig * covMats[iter->first]);

                    Eigen::MatrixXd rankMew(dims[iter->first], dims[iter->first]);
                    rankMew = Eigen::MatrixXd::Zero(dims[iter->first], dims[iter->first]);

                    for(uint i = 0; i < mParameters.parentSize; ++i){
                        rankMew += mParameters.weights(i, 0) * yilambdas[i] * yilambdas[i].transpose();
                    }

                    covMats[iter->first] = mem + rankOne + rankMew;

                    //update the eigen decomposition of the covariance matrix
                    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covMats[iter->first]);
                    eigenVectors[iter->first] = eigenSolver.eigenvectors();
                    eigenValuesSqrt[iter->first] = eigenSolver.eigenvalues();
                    for(int k = 0; k < dims[iter->first]; ++k) {
                        eigenValuesSqrt[iter->first](k, 0) = sqrt(eigenValuesSqrt[iter->first](k, 0));
                    }

                    //update mean
                    currMeans[iter->first] = newMean;

                    //generate offspring
                    cout << "Generating offspring" << endl;
                    generateOffspring(iter->second, eigenVectors[iter->first], eigenValuesSqrt[iter->first], currMeans[iter->first], stepSizes[iter->first], dims[iter->first]);
                }
                //evaluate offspring
                cout << "Evaluating Fitness" << endl;
                evaluateCompetitiveFitness();
                
                cout << "Time taken for this generation : " << time(0) - t << endl;
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
    //read in xml doc describing ANN
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return false;
    }
    pugi::xml_node root = doc.first_child();

    //make pseudo chrom to figure out how many dimensions
    Chromosome* chrom = new NNChromosome();
    if(!chrom->initialize(&root)){
        cerr << "Error: unable to create chromosome in GA" << endl;
        return false;
    }
    int dims = calcDims(chrom);
    delete chrom;

    //set initial variables
    mParameters.populationSize = 4 + 3 * log((double)dims);
    mParameters.parentSize = mParameters.populationSize / 2;

    MPI_Comm_size(MPI_COMM_WORLD, &mTotalSlaveProcs);
    if(mTotalSlaveProcs == 0)
        return false;

    int totalWork = mParameters.evalsPerCompChrom * mParameters.populationSize;
    mTotalRequests = totalWork > mTotalSlaveProcs ? mTotalSlaveProcs : totalWork;

    mRequests = new MPI_Request[mTotalRequests];
    mRetrievedFitnesses = new double[mTotalRequests];

    setupWeights();
    
    //determine how many teams
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

    //if 1 team, initialize normal population, otherwise init competitive populations first(normal population will be constructed once runDeltaCodes() is called by the algorithm
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
            //manually construct the neural nets, concat them as needed into the different teams and then pass them to the populations
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

    return true;
}

void CMAES::setupWeights(){
    mParameters.weights.resize(mParameters.parentSize, 1);
    mParameters.mewEff = 0;

    double total = 0;
    double mewbar = (double)(mParameters.populationSize) / 2.;

    for(uint k = 1; k <= mParameters.parentSize; ++k){
        mParameters.weights(k - 1, 0) = log(mewbar + 0.5) - log((double)k);
        total += mParameters.weights(k - 1, 0);
    }

    for(uint k = 0; k < mParameters.parentSize; ++k){
        mParameters.weights(k, 0) = mParameters.weights(k, 0) / total;
        mParameters.mewEff += mParameters.weights(k, 0) * mParameters.weights(k, 0);
    }

    mParameters.mewEff = 1 / mParameters.mewEff;
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

    quicksort(_population, 0, _population.size() - 1);
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

    for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
        quicksort(iter->second, 0, iter->second.size() - 1);
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
        newChrom->addDelta(newWeights);
        mPopulation.push_back(newChrom);
    }

    for(map<int, vector<Chromosome*>>::iterator iter = mCompetitivePopulations.begin(); iter != mCompetitivePopulations.end(); ++iter){
        for(uint k = 0; k < iter->second.size(); ++k){
            delete iter->second[k];
        }
    }

    mCompetitivePopulations.clear();
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

void CMAES::quicksort(vector<Chromosome*>& elements, int left, int right)
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

int CMAES::calcDims(Chromosome* _chrom){
    int dims = 0;
    vector<map<uint, vector<double>>> p0w = _chrom->getWeightData();

    for(uint k = 0; k < p0w.size(); ++k){
        for(map<uint, vector<double>>::iterator iter = p0w[k].begin(); iter != p0w[k].end(); ++iter){
            dims += iter->second.size();
        }
    }

    return dims;
}

void CMAES::calcMean(const vector<Chromosome*>& _population, Eigen::MatrixXd& _weightedMean, int _dims, vector<Eigen::MatrixXd>& _yilambdas, double _stepSize){
    assert(_population.size() >= mParameters.parentSize);
    
    for(uint k = 0; k < _dims; ++k){
        _weightedMean(k, 0) = 0;
    }

    //calculates new mean, adds yilambdas to vector of matrices as they are needed to calculate new covariance matrix
    for(uint k = 0; k < mParameters.parentSize; ++k){
        int currPos = 0;
        Eigen::MatrixXd yilambda(_dims, 1);

        vector<map<uint, vector<double>>> pw = _population[k]->getWeightData();

        for(uint l = 0; l < pw.size(); ++l){
            for(map<uint, vector<double>>::iterator iter = pw[l].begin(); iter != pw[l].end(); ++iter){
                for(uint i = 0; i < iter->second.size(); ++i){
                    yilambda(currPos, 0) = (iter->second[i] - _weightedMean(currPos, 0)) / _stepSize;
                    _weightedMean(currPos, 0) += mParameters.weights(k, 0) * iter->second[i];
                    
                    currPos++;
                }
            }
        }

        _yilambdas.push_back(yilambda);
    }
}

void CMAES::generateOffspring(vector<Chromosome*>& _population, const Eigen::MatrixXd& _eigenVectors, const Eigen::MatrixXd& _eigenValuesSqrt, Eigen::MatrixXd& _means, double _stepSize, int _dims){
    boost::mt19937 rng(rand());
    boost::normal_distribution<double> dist;
    boost::variate_generator<boost::mt19937& ,boost::normal_distribution<double>> gen(rng, dist);

    vector<Chromosome*> offspring;

    for(uint k = 0; k < mParameters.populationSize; ++k){
        //sample from univariate, sample
        Eigen::MatrixXd sampleTemp(_dims, 1);
        Eigen::MatrixXd sample(_dims, 1);

        //keeps track of dims which fall out of search space, these need to be resampled until they are feasible
        vector<int> infeasiblePositions;
        for (int i = 0; i < _dims ; ++i) {
            infeasiblePositions.push_back(i);
        }

        for(int i = 0; i < _dims; ++i) {
            sampleTemp(i, 0) = gen() * _eigenValuesSqrt(i, 0);
        }
        sample = _eigenVectors * sampleTemp;

        for(uint i = 0; i < _dims; ++i){
            sample(i, 0) = _stepSize * sample(i, 0) + _means(i, 0);

            /*if(sample(i, 0) < -1)
                sample(i, 0) = -1;
            else if(sample(i, 0) > 1)
                sample(i, 0) = 1;*/
        }

        //set generated synapses for new child
        vector<map<uint, vector<double>>> generatedVec = _population[0]->getWeightData();
        int currPos = 0;

        for(uint i = 0; i < generatedVec.size(); ++i){
            for(map<uint, vector<double>>::iterator iter = generatedVec[i].begin(); iter != generatedVec[i].end(); ++iter){
                for(uint l = 0; l < iter->second.size(); ++l){
                    iter->second[l] = sample(currPos++, 0);
                }
            }
        }

        Chromosome* child = _population[0]->clone();
        child->setWeights(generatedVec);
        offspring.push_back(child);
    }

    //delete old population, set new population to current
    for(uint k = 0; k < _population.size(); ++k)
        delete _population[k];
    _population.clear();

    _population = offspring;
}