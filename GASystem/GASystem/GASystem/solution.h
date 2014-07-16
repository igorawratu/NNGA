#ifndef SOLUTION_H
#define SOLUTION_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "neuralnetwork.h"
#include "common.h"

using namespace std;

typedef pair<uint, double> CompetitiveFitness;

class Solution
{
public:
    Solution(){}
    Solution(string _filename);
    Solution(vector<NeuralNetwork>);
    Solution(int* _nodes, int* _format, double* _weights, int _formatSize, int _weightSize);
    
    vector<vector<double>> evaluateAllNeuralNetworks(vector<map<uint, double>> _inputs);
    vector<double> evaluateNeuralNetwork(uint _index, map<uint, double> _inputs);
    vector<double> evaluateNeuralNetwork(uint _index, map<uint, double> _inputs, uint _teamID);
    void printToFile(string _filename);

    double& fitness(){return mFitness;}
    double& realFitness(){return mRealFitness;}
	vector<CompetitiveFitness>& competitiveFitness(){return mCompFitness;}

    void serialize(int*& _nodes, int*& _format, double*& _weights, int& _nodeSize, int& _formatSize, int& _weightSize);

private:
    vector<NeuralNetwork> mNeuralNets;
    double mFitness, mRealFitness;
	vector<CompetitiveFitness> mCompFitness;
};

#endif