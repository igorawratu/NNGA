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
    Solution(vector<NeuralNetwork>);
    
    vector<vector<double>> evaluateAllNeuralNetworks(vector<map<uint, double>> _inputs);
    vector<double> evaluateNeuralNetwork(uint _index, map<uint, double> _inputs);
    void printToFile(string _filename);

    double& fitness(){return mFitness;}
    double& realFitness(){return mRealFitness;}

private:
    Solution(){}

private:
    vector<NeuralNetwork> mNeuralNets;
    double mFitness, mRealFitness;
};

#endif