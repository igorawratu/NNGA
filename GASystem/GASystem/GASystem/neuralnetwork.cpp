#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork(xmldoc* _file, bool _checkLoops){
    mCounter = -1;

    constructNNStructure(_file, true, _checkLoops);
}

//change this to neuroinfo
NeuralNetwork::NeuralNetwork(xmldoc* _file, map<uint, vector<double>> _weights){
    mCounter = -1;
    
    constructNNStructure(_file, false, false);

    for(map<uint, vector<double>>::iterator iter = _weights.begin(); iter != _weights.end(); iter++)
        mNeuronCache[iter->first]->setWeights(iter->second);

}

void NeuralNetwork::constructNNStructure(xmldoc* _file, bool _withWeights, bool _checkLoops){
    pugi::xml_node travNode = _file->child("NeuralNetwork");
    map<uint, set<uint>> predecessorMap;
    boost::mt19937 rng(rand());

    for(pugi::xml_node node = travNode.first_child(); node; node = node.next_sibling()){
        uint neuronID = atoi(node.attribute("ID").value());
        Neuron* neuron;

        //if input type, create leaf neuron, otherwise create a nonleaf neuron
        if(strcmp(node.attribute("Type").value(), "Input"))
            neuron = new LeafNeuron(&mNeuronCache, vector<double>());
        else{
            vector<double> weights;
            set<uint> predecessors;

            //assign activation function
            ActivationFunction activationFunction;
            if(strcmp(node.attribute("ActivationFunction").value(), "Sigmoid"))
                activationFunction = SIGMOID;
            else{
                    cerr << "Error: unable to understand the activation function of neuron " << neuronID << ", defaulting to sigmoid" << endl;
                    activationFunction = SIGMOID;
            }

            //assign predecessors
            pugi::xml_node predecessorsRoot = node.child("Predecessors");
            for(pugi::xml_node predecessorNode = predecessorsRoot.first_child(); predecessorNode; predecessorNode = predecessorNode.next_sibling())
                predecessors.insert(atoi(predecessorNode.attribute("ID").value()));
            predecessorMap[neuronID] = predecessors;

            //assign weights
            if(_withWeights){
                pugi::xml_node weightRoot = node.child("Weights");
                if(strcmp(weightRoot.attribute("dist").value(), "Fixed")){
                    for(pugi::xml_node weightNode = weightRoot.first_child(); weightNode; weightNode = weightNode.next_sibling())
                        weights.push_back(atof(weightNode.attribute("value").value()));
                }
                else{
                    //set the amount of weights needed to deal with the inputs, +1 for the bias
                    uint weightCount = predecessors.size() + 1;

                    //initialize random weights
                    if(strcmp(weightRoot.attribute("dist").value(), "Uniform")){
                        boost::uniform_real<double> weightDist(atoi(weightRoot.attribute("min").value()), atoi(weightRoot.attribute("max").value()));
                        boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);                  

                        for(int k = 0; k < weightCount; k++)
                            weights.push_back(genWeight());
                    }
                    else{
                        cerr << "Error: Invalid distribution found for neuron " << neuronID << ", defaulting to Uniform" << endl;
                        boost::uniform_real<double> weightDist(atoi(weightRoot.attribute("min").value()), atoi(weightRoot.attribute("max").value()));
                        boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);                  

                        for(int k = 0; k < weightCount; k++)
                            weights.push_back(genWeight());
                    }
                }
            }
            neuron = new NonLeafNeuron(&mNeuronCache, weights, activationFunction);

            if(strcmp(node.attribute("Type").value(), "Output"))
                mOutput[neuronID] = neuron;

        }

        mNeuronCache[neuronID] = neuron;
    }

    //link the predecessors
    for(map<uint, set<uint>>::iterator iter = predecessorMap.begin(); iter != predecessorMap.end(); iter++)
        mNeuronCache[iter->first]->setInput(iter->second, _checkLoops);
}

NeuralNetwork::NeuralNetwork(const NeuralNetwork& _other){
    mCounter = -1;
    
    map<uint, Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;


    for(map<uint, Neuron*>::const_iterator iter = _other.mNeuronCache.begin(); iter != _other.mNeuronCache.end(); iter++){
        mNeuronCache[iter->first] = iter->second->clone();
        mNeuronCache[iter->first]->setNeuronCache(&mNeuronCache);
    }

    for(map<uint, Neuron*>::const_iterator iter = _other.mOutput.begin(); iter!= _other.mOutput.end(); iter++)
        mOutput[iter->first] = mNeuronCache[iter->first];
}

NeuralNetwork& NeuralNetwork::operator = (const NeuralNetwork& _other){
    mCounter = -1;
    
    map<uint, Neuron*> mOutput;
    map<uint, Neuron*> mNeuronCache;


    for(map<uint, Neuron*>::const_iterator iter = _other.mNeuronCache.begin(); iter != _other.mNeuronCache.end(); iter++){
        mNeuronCache[iter->first] = iter->second->clone();
        mNeuronCache[iter->first]->setNeuronCache(&mNeuronCache);
    }

    for(map<uint, Neuron*>::const_iterator iter = _other.mOutput.begin(); iter!= _other.mOutput.end(); iter++)
        mOutput[iter->first] = mNeuronCache[iter->first];
    
    return *this;
}
    
NeuralNetwork::~NeuralNetwork(){
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second)
        {
            delete iter->second;
            iter->second = 0;
        }
    }
    mNeuronCache.clear();

    mOutput.clear();
}

vector<double> NeuralNetwork::evaluate(map<uint, double> _inputs){
    vector<double> output;

    for(map<uint, double>::iterator iter = _inputs.begin(); iter != _inputs.end(); iter++){
        map<uint, Neuron*>::const_iterator neuronIter = mNeuronCache.find(iter->first);
        if(neuronIter == mNeuronCache.end())
        {
            cerr << "Error: unable to find input node " << iter->first << ", neuron evaluation will now terminate" << endl;
            return output;
        }

        mNeuronCache[iter->first]->setInput(iter->second);
    }

    mCounter++;
    for(map<uint, Neuron*>::iterator iter = mOutput.begin(); iter != mOutput.end(); iter++)
        output.push_back(iter->second->evaluate(mCounter));

    return output;
}

void NeuralNetwork::getStructure(xmldoc& _doc){
    if(!_doc.empty()){
        cout << "Error: please pass through an empty xml doc" << endl;
        return;
    }

    pugi::xml_node root = _doc.append_child("NeuralNetwork");
    
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        pugi::xml_node currentNeuron = root.append_child("Neuron");
        
        //set neuron ID
        currentNeuron.append_attribute("ID") = iter->first;

        if(iter->second->getNeuronType() == LEAF)
            //set neuron type
            currentNeuron.append_attribute("Type") = "Input";
        else{
            map<uint, Neuron*>::const_iterator neuronIter = mOutput.find(iter->first);

            //set neuron type
            if(neuronIter == mOutput.end())
                currentNeuron.append_attribute("Type") = "Hidden";
            else currentNeuron.append_attribute("Type") = "Output";
        
            //set activation function type
            switch(iter->second->getActivationFunction()){
                case SIGMOID:
                    currentNeuron.append_attribute("ActivationFunction") = "Sigmoid";
                    break;
                default:
                    break;
            }

            //set weight data
            pugi::xml_node weightRoot = currentNeuron.append_child("Weights");
            vector<double> weights = iter->second->getWeights();
            for(int k = 0; k < weights.size(); k++)
                weightRoot.append_child("Weight").append_attribute("value") = weights[k];

            //set predecessor data
            pugi::xml_node predecessorRoot = currentNeuron.append_child("Predecessors");
            set<uint> predecessors = iter->second->getPredecessors();
            for(set<uint>::iterator predIter = predecessors.begin(); predIter != predecessors.end(); predIter++)
                predecessorRoot.append_child("Predecessor").append_attribute("ID") = *predIter;

        }
    }
}