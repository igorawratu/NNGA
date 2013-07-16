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
    Solution(string _filename);
    Solution(vector<map<uint, NeuronInfo>>);
    
    vector<vector<double>> evaluateAllNeuralNetworks();
    vector<double> evaluateNeuralNetwork(uint _index);

    double& fitness(){return mFitness;}

private:
    Solution(){}

private:
    vector<NeuralNetwork> mNeuralNets;
    double mFitness;
};

#endif