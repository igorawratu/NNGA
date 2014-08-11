#ifndef CMAES_H
#define CMAES_H

#include "geneticalgorithm.h"
#include "cmaesparameters.h"
#include "nnchromosome.h"

#include "common.h"
#include "workstatus.h"
#include <vector>
#include <map>
#include <mpi.h>
#include "multivariatenormal.h"
#include <math.h> 

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

#include <Eigen/Dense>

class CMAES : public GeneticAlgorithm{
public:
    CMAES(CMAESParameters _parameters);
    virtual ~CMAES();

    virtual Solution train(SimulationContainer* _simulationContainer, string _outputFileName);

private:
    void evaluateFitness(vector<Chromosome*>& _population);
    void evaluateCompetitiveFitness();
    void runDeltaCodes();
    void stopSlaves();
    void hostwork();
    void sendCompData(Solution* _solution, int _slave);
    void sendData(Solution* _solution, int _slave);
    Solution* constructCompSolution(map<int, vector<pair<uint, uint>>>& _competitiveTracklist, vector<int>& _team, vector<uint>& _position);
    bool setup();
    void quicksort(vector<Chromosome*>& elements, int left, int right);
    void calcMean(const vector<Chromosome*>& _population, Eigen::MatrixXd& _weightedMean, int _dims, vector<Eigen::MatrixXd>& _yilambdas, double _stepSize);
    void setupWeights();
    int calcDims(Chromosome* _chrom);
    void generateOffspring(vector<Chromosome*>& _population, const Eigen::MatrixXd& _eigenVectors, const Eigen::MatrixXd& _eigenValuesSqrt, Eigen::MatrixXd& _means, double _stepSize, int _dims);

private:
    CMAESParameters mParameters;
    uint mNumTeams;
	int* mUpdateList;
    MPI_Request *mRequests, *mTeamRequests;
    double* mRetrievedFitnesses, *mRetrievedCompetitiveFitnesses;
    int* mRetrievedTeamIDs;
    int mTotalSlaveProcs, mTotalRequests;
    volatile WorkStatus mWorkStatus;
    SimulationContainer* mSimulationContainer;
	uint mStages, mStage;
    Solution* mSavedSlaveSolution;
    std::vector<Chromosome*> mPopulation;
    std::map<int, vector<Chromosome*>> mCompetitivePopulations;
    double mEff;

private:
    CMAES(const CMAES& _other){}
    CMAES(){}
};

#endif