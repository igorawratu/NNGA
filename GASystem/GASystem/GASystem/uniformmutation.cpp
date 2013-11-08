#include "uniformmutation.h"


UniformMutation::UniformMutation(){}
UniformMutation::~UniformMutation(){}

void UniformMutation::execute(vector<double>& _weights, map<string, double>& _parameters){
    double mutationProbability, deviation, maxConstraint, minConstraint;

    if(!getParameter<double>(_parameters, mutationProbability, "MutationProbability"))
        return;

    if(!getParameter<double>(_parameters, deviation, "Deviation"))
        return;

    //change the constraints to work on the NN side instead
    if(!getParameter<double>(_parameters, maxConstraint, "MaxConstraint"))
        return;

    if(!getParameter<double>(_parameters, minConstraint, "MinConstraint"))
        return;

    assert(deviation > 0);

    boost::mt19937 mRNGMutationProb(rand()), mRNGMutation(rand());

    boost::uniform_real<double> mutationProbDist(0, 1);
    boost::uniform_real<double> mutationDist(-deviation, deviation);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genMutationProb(mRNGMutationProb, mutationProbDist);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genMutation(mRNGMutation, mutationDist);

    for(uint k = 0; k < _weights.size(); k++){
        if(genMutationProb() < mutationProbability)
            _weights[k] += genMutation();
        if(_weights[k] > maxConstraint)
            _weights[k] = maxConstraint;
        if(_weights[k] < minConstraint)
            _weights[k] = minConstraint;
    }
}