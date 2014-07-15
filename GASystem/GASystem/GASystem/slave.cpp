#include "slave.h"

Slave::Slave(Simulation* _sim) : mSimulationContainer(_sim){
}

void Slave::run(){
    while(true){
        int initialDat[4];

        int weightCount, formatCount, nodeCount;
        MPI_Status status;

        MPI_Recv(&initialDat[0], 4, MPI_INT, 0, 1, MPI_COMM_WORLD, &status);

        nodeCount = initialDat[0];
        formatCount = initialDat[1];
        weightCount = initialDat[2];

        if(nodeCount < 0)
            break;

        double* weights = new double[weightCount];
        int* format = new int[formatCount];
        int* nodes = new int[nodeCount];

        MPI_Recv(nodes, nodeCount, MPI_INT, 0, 1, MPI_COMM_WORLD, &status);
        MPI_Recv(format, formatCount, MPI_INT, 0, 1, MPI_COMM_WORLD, &status);
        MPI_Recv(weights, weightCount, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD, &status);

        Solution solution(nodes, format, weights, formatCount, weightCount);
        mSimulationContainer.runFullSimulation(&solution);
        mSimulationContainer.resetSimulation();

        if(initialDat[3] == 0){
            vector<CompetitiveFitness> compFitnesses = solution.competitiveFitness();
            
            int* teamIDs = new int[compFitnesses.size()];
            double* fitnesses = new double[compFitnesses.size()];

            for(uint k = 0; k < compFitnesses.size(); ++k){
                teamIDs[k] = compFitnesses[k].first;
                fitnesses[k] = compFitnesses[k].second;
            }

            MPI_Send(fitnesses, compFitnesses.size(), MPI_DOUBLE, 0, 1, MPI_COMM_WORLD);
            MPI_Send(teamIDs, compFitnesses.size(), MPI_INT, 0, 1, MPI_COMM_WORLD);

            delete [] teamIDs;
            delete [] fitnesses;
        }
        else{
            double realFitness = solution.realFitness();
            double fitness = solution.fitness();

            double out[2];
            out[0] = realFitness;
            out[1] = fitness;

            MPI_Send(&out[0], 2, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD);
        }
    }
}