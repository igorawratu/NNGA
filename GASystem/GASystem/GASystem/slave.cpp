#include "slave.h"

Slave::Slave(Simulation* _sim) : mSimulationContainer(_sim){
}

void Slave::run(){
    while(true){
        int initialDat[3];

        int weightCount, formatCount, nodeCount;
        MPI_Status status;

        MPI_Recv(&initialDat[0], 3, MPI_INT, 0, 0, MPI_COMM_WORLD, &status);
        nodeCount = initialDat[0];
        formatCount = initialDat[1];
        weightCount = initialDat[2];

        if(formatCount < 0)
            break;

        double* weights = new double[weightCount];
        int* format = new int[formatCount];
        int* nodes = new int[nodeCount];

        MPI_Recv(nodes, nodeCount, MPI_INT, 0, 0, MPI_COMM_WORLD, &status);
        MPI_Recv(format, formatCount, MPI_INT, 0, 0, MPI_COMM_WORLD, &status);
        MPI_Recv(weights, weightCount, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, &status);

        Solution solution(nodes, format, weights, formatCount, weightCount);
        mSimulationContainer.runFullSimulation(&solution);
        mSimulationContainer.resetSimulation();

        double realFitness = solution.realFitness();
        double fitness = solution.fitness();

        double out[2];
        out[0] = realFitness;
        out[1] = fitness;

        MPI_Send(&out[0], 2, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
    }
}