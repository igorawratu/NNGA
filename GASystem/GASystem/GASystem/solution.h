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

class Solution
{
public:
    Solution(){}
    Solution(string _filename);
    Solution(vector<NeuralNetwork>);
    Solution(int* _nodes, int* _format, float* _weights, int _formatSize, int _weightSize);
    
    vector<vector<double>> evaluateAllNeuralNetworks(vector<map<uint, double>> _inputs);
    vector<double> evaluateNeuralNetwork(uint _index, map<uint, double> _inputs);
    void printToFile(string _filename);

    double& fitness(){return mFitness;}
    double& realFitness(){return mRealFitness;}

    void serialize(int*& _nodes, int*& _format, float*& _weights, int& _formatSize, int& _weightSize);

private:
    vector<NeuralNetwork> mNeuralNets;
    double mFitness, mRealFitness;
};

#endif