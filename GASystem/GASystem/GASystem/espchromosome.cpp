#include "espchromosome.h"

ESPChromosome::ESPChromosome(){
    mSSMax = 1;
    mSSMin = -1;
    mFitness = 0;
    mRealFitness = 0;
}

void ESPChromosome::reInitialize(){
    uint weightCount = mNeuron->getWeights().size();
    vector<double> weights;

    boost::mt19937 rng(rand());
    boost::uniform_real<double> weightDist(mSSMin, mSSMax);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);      
    for(uint k = 0; k < weightCount; k++)
        weights.push_back(genWeight());
    mNeuron->setWeights(weights);
    mFitness = 0;
    mRealFitness = 0;
}

bool ESPChromosome::initialize(pugi::xml_node* _root){
    vector<double> weights;
    set<uint> predecessors;

    //assign activation function
    if(_root->attribute("ActivationFunction").empty()){
        cerr << "Error: Non input nodes must have an Activation Function" << endl;
        return false;
    }

    ActivationFunction activationFunction;
    if(strcmp(_root->attribute("ActivationFunction").value(), "Sigmoid") == 0)
        activationFunction = SIGMOID;
    else{
            cerr << "Error: unable to understand the activation function of a neuron, defaulting to sigmoid" << endl;
            activationFunction = SIGMOID;
    }

    //assign predecessors
    if(_root->child("Predecessors").empty()){
        cerr << "Error: Non input nodes must have Predecessors" << endl;
        return false;
    }

    pugi::xml_node predecessorsRoot = _root->child("Predecessors");
    for(pugi::xml_node predecessorNode = predecessorsRoot.first_child(); predecessorNode; predecessorNode = predecessorNode.next_sibling()){
        if(predecessorNode.attribute("ID").empty()){
            cerr << "Error: Predecessors must have an ID" << endl;
            return false;
        }
        predecessors.insert(atoi(predecessorNode.attribute("ID").value()));
    }

    //assign weights
    if(_root->child("Weights").empty()){
        cerr << "Error: Non input nodes must have Weights" << endl;
        return false;
    }

    pugi::xml_node weightRoot = _root->child("Weights");

    if(weightRoot.attribute("Distribution").empty()){
        cerr << "Error: Weights must have a Distribution attribute" << endl;
        return false;
    }

    if(strcmp(weightRoot.attribute("Distribution").value(), "Fixed") == 0){
        for(pugi::xml_node weightNode = weightRoot.first_child(); weightNode; weightNode = weightNode.next_sibling()){
            if(weightNode.attribute("Value").empty()){
                cerr << "Error: Each weight must have a Value attribute if the Distribution is Fixed" << endl;
                return false;
            }
     
            weights.push_back(atof(weightNode.attribute("Value").value()));
        }
    }
    else{
        //create the rng
        boost::mt19937 rng(rand());
        //set the amount of weights needed to deal with the inputs, +1 for the bias
        uint weightCount = predecessors.size() + 1;

        //initialize random weights
        if(strcmp(weightRoot.attribute("Distribution").value(), "Uniform") == 0){
            if(weightRoot.attribute("Min").empty()){
                cerr << "Error: cannot find Min attribute for weight in xml" << endl;
                return false;
            }
            if(weightRoot.attribute("Max").empty()){
                cerr << "Error: cannot find Max attribute for weight in xml" << endl;
                return false;
            }

            mSSMin = atof(weightRoot.attribute("Min").value());
            mSSMax = atof(weightRoot.attribute("Max").value());

            boost::uniform_real<double> weightDist(mSSMin, mSSMax);
            boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);      
            for(uint k = 0; k < weightCount; k++)
                weights.push_back(genWeight());
        }
        else{
            cerr << "Error: Invalid distribution found for a neuron, defaulting to Uniform" << endl;
            boost::uniform_real<double> weightDist(atoi(weightRoot.attribute("Min").value()), atoi(weightRoot.attribute("Max").value()));
            boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);                  

            for(uint k = 0; k < weightCount; k++)
                weights.push_back(genWeight());
        }
    }

    mNeuron = new NonLeafNeuron(NULL, weights, activationFunction);
    mNeuron->setInput(predecessors, true);

    return true;
}

ESPChromosome::ESPChromosome(const ESPChromosome& other){
    mNeuron = new NonLeafNeuron(*other.mNeuron);
    mSSMax = other.mSSMax;
    mSSMin = other.mSSMin;
    mFitness = 0;
    mRealFitness = 0;
}

ESPChromosome::~ESPChromosome(){
    if(mNeuron)
        delete mNeuron;
}

ESPChromosome& ESPChromosome::operator = (const ESPChromosome& other){
    if(mNeuron)
        delete mNeuron;

    mNeuron = new NonLeafNeuron(*other.mNeuron);
    mSSMax = other.mSSMax;
    mSSMin = other.mSSMin;
    mFitness = 0;
    mRealFitness = 0;

    return *this;
}

void ESPChromosome::mutate(string _mutationType, map<string, double>& _parameters){
    Mutation* mutationAlgorithm = MutationFactory::instance().create(_mutationType);

    vector<double> weights = mNeuron->getWeights();
    mutationAlgorithm->execute(weights, _parameters);
    mNeuron->setWeights(weights);

    delete mutationAlgorithm;
}

vector<map<uint, vector<double>>> ESPChromosome::getWeightData(){
    vector<map<uint, vector<double>>> out;
    out.push_back(map<uint, vector<double>>());
    out[0][1] = mNeuron->getWeights();

    return out;
}

bool ESPChromosome::addDelta(vector<map<uint, vector<double>>> _weights){
    vector<double> newWeights;
    vector<double> currentNeuronWeights = mNeuron->getWeights();
    vector<double> deltaWeights = _weights[0][1];

    if(deltaWeights.size() != currentNeuronWeights.size())
        return false;

    for(uint k = 0; k < currentNeuronWeights.size(); ++k)
        newWeights.push_back(deltaWeights[k] + currentNeuronWeights[k]);

    mNeuron->setWeights(newWeights);

    return true;
}

vector<map<uint, NeuronInfo>> ESPChromosome::getFullStructureData(){
    cout << "cannot get structure of ESP chromosome" << endl;

    return vector<map<uint, NeuronInfo>>();
}

void ESPChromosome::setWeights(vector<map<uint, vector<double>>>& _weights){
    for(uint k = 0; k < _weights.size(); ++k)
        for(map<uint, vector<double>>::iterator iter = _weights[k].begin(); iter != _weights[k].end(); ++iter)
            mNeuron->setWeights(iter->second);
}

void ESPChromosome::setStructure(vector<map<uint, NeuronInfo>>& structure){
    cout << "structure setting is turned off for ESP chromosomes" << endl;
}

Chromosome* ESPChromosome::clone(){
    return new ESPChromosome(*this);
}

void ESPChromosome::setTeamID(uint _id){
    mTeamID = _id;
}

uint ESPChromosome::getTeamID(){
    return mTeamID;
}