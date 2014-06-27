#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork(){
    mCounter = -1;
}

bool NeuralNetwork::initialize(pugi::xml_node* _nnRoot, bool _checkLoops){
    return constructNNStructure(_nnRoot, _checkLoops);
}

//change this to neuroinfo
NeuralNetwork::NeuralNetwork(map<uint, NeuronInfo> _neuronInfo){

    for(map<uint, NeuronInfo>::iterator iter = _neuronInfo.begin(); iter != _neuronInfo.end(); iter++){
        if(iter->second.neuronType == LEAF)
            mNeuronCache[iter->first] = new LeafNeuron(&mNeuronCache, vector<double>());
        else{
            Neuron* currNeuron = new NonLeafNeuron(&mNeuronCache, iter->second.weights, iter->second.activationFunction);
            currNeuron->setInput(iter->second.predecessors, false);
            mNeuronCache[iter->first] = currNeuron;
            if(iter->second.neuronType == OUTPUT)
                mOutput[iter->first] = currNeuron;
        }
    }

}

void NeuralNetwork::setStructure(map<uint, Neuron*> _neuronCache, map<uint, Neuron*> _output){
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second)
        {
            delete iter->second;
            iter->second = 0;
        }
    }
    mNeuronCache.clear();
    mOutput.clear();

    mCounter = 0;
    for(map<uint, Neuron*>::iterator iter = _neuronCache.begin(); iter != _neuronCache.end(); ++iter){
        mNeuronCache[iter->first] = iter->second->clone();
        for(map<uint, Neuron*>::iterator outIter = _output.begin(); outIter != _output.end(); ++outIter){
            if(iter->second == outIter->second){
                mOutput[iter->first] = mNeuronCache[iter->first];
                break;
            }
        }
        mNeuronCache[iter->first]->setNeuronCache(&mNeuronCache);
    }
}

void NeuralNetwork::setStructure(map<uint, NeuronInfo> _neuronInfo){
    //clear old structure
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second)
        {
            delete iter->second;
            iter->second = 0;
        }
    }
    mNeuronCache.clear();
    mOutput.clear();

    for(map<uint, NeuronInfo>::iterator iter = _neuronInfo.begin(); iter != _neuronInfo.end(); iter++){
        if(iter->second.neuronType == LEAF)
            mNeuronCache[iter->first] = new LeafNeuron(&mNeuronCache, vector<double>());
        else{
            Neuron* currNeuron = new NonLeafNeuron(&mNeuronCache, iter->second.weights, iter->second.activationFunction);
            currNeuron->setInput(iter->second.predecessors, false);
            mNeuronCache[iter->first] = currNeuron;
            if(iter->second.neuronType == OUTPUT)
                mOutput[iter->first] = currNeuron;
        }
    }
}

bool NeuralNetwork::constructNNStructure(pugi::xml_node* _nnRootNode, bool _checkLoops){
    map<uint, set<uint>> predecessorMap;

    for(pugi::xml_node node = _nnRootNode->first_child(); node; node = node.next_sibling()){
        if(node.attribute("ID").empty()){
            cerr << "Error: node does not have ID" << endl;
            return false;
        }
        
        uint neuronID = atoi(node.attribute("ID").value());
        Neuron* neuron;

        if(node.attribute("Type").empty()){
            cerr << "Error: node does not have a Type" << endl;
            return false;
        }

        //if input type, create leaf neuron, otherwise create a nonleaf neuron
        if(strcmp(node.attribute("Type").value(), "Input") == 0)
            neuron = new LeafNeuron(&mNeuronCache, vector<double>());
        else{
            vector<double> weights;
            set<uint> predecessors;

            //assign activation function
            if(node.attribute("ActivationFunction").empty()){
                cerr << "Error: Non input nodes must have an Activation Function" << endl;
                return false;
            }

            ActivationFunction activationFunction;
            if(strcmp(node.attribute("ActivationFunction").value(), "Sigmoid") == 0)
                activationFunction = SIGMOID;
            else{
                    cerr << "Error: unable to understand the activation function of neuron " << neuronID << ", defaulting to sigmoid" << endl;
                    activationFunction = SIGMOID;
            }

            //assign predecessors
            if(node.child("Predecessors").empty()){
                cerr << "Error: Non input nodes must have Predecessors" << endl;
                return false;
            }

            pugi::xml_node predecessorsRoot = node.child("Predecessors");
            for(pugi::xml_node predecessorNode = predecessorsRoot.first_child(); predecessorNode; predecessorNode = predecessorNode.next_sibling()){
                if(predecessorNode.attribute("ID").empty()){
                    cerr << "Error: Predecessors must have an ID" << endl;
                    return false;
                }
                predecessors.insert(atoi(predecessorNode.attribute("ID").value()));
            }
            predecessorMap[neuronID] = predecessors;

            //assign weights
            if(node.child("Weights").empty()){
                cerr << "Error: Non input nodes must have Weights" << endl;
                return false;
            }

            pugi::xml_node weightRoot = node.child("Weights");

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

                    boost::uniform_real<double> weightDist(atof(weightRoot.attribute("Min").value()), atof(weightRoot.attribute("Max").value()));
                    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);      
                    for(uint k = 0; k < weightCount; k++)
                        weights.push_back(genWeight());
                }
                else{
                    cerr << "Error: Invalid distribution found for neuron " << neuronID << ", defaulting to Uniform" << endl;
                    boost::uniform_real<double> weightDist(atoi(weightRoot.attribute("Min").value()), atoi(weightRoot.attribute("Max").value()));
                    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genWeight(rng, weightDist);                  

                    for(uint k = 0; k < weightCount; k++)
                        weights.push_back(genWeight());
                }
            }

            neuron = new NonLeafNeuron(&mNeuronCache, weights, activationFunction);

            if(strcmp(node.attribute("Type").value(), "Output") == 0)
                mOutput[neuronID] = neuron;

        }

        mNeuronCache[neuronID] = neuron;
    }

    //link the predecessors
    for(map<uint, set<uint>>::iterator iter = predecessorMap.begin(); iter != predecessorMap.end(); iter++)
        if(!mNeuronCache[iter->first]->setInput(iter->second, _checkLoops))
            return false;

    return true;
}

NeuralNetwork::NeuralNetwork(const NeuralNetwork& _other){

    for(map<uint, Neuron*>::const_iterator iter = _other.mNeuronCache.begin(); iter != _other.mNeuronCache.end(); iter++){
        mNeuronCache[iter->first] = iter->second->clone();
        mNeuronCache[iter->first]->setNeuronCache(&mNeuronCache);
    }

    for(map<uint, Neuron*>::const_iterator iter = _other.mOutput.begin(); iter!= _other.mOutput.end(); iter++)
        mOutput[iter->first] = mNeuronCache[iter->first];
}

NeuralNetwork& NeuralNetwork::operator = (const NeuralNetwork& _other){
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second)
        {
            delete iter->second;
            iter->second = 0;
        }
    }
    mNeuronCache.clear();
    mOutput.clear();

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

    for(map<uint, Neuron*>::iterator iter = mOutput.begin(); iter != mOutput.end(); iter++)
        output.push_back(iter->second->evaluate(++mCounter));

    return output;
}

void NeuralNetwork::getXMLStructure(pugi::xml_node& _root){
    pugi::xml_node nnRoot = _root.append_child("NeuralNetwork");
    
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        pugi::xml_node currentNeuron = nnRoot.append_child("Neuron");
        
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
            pugi::xml_attribute weightAtt = weightRoot.append_attribute("Distribution");
            weightAtt.set_value("Fixed");
            vector<double> weights = iter->second->getWeights();
            for(uint k = 0; k < weights.size(); k++)
                weightRoot.append_child("Weight").append_attribute("Value") = boost::lexical_cast<std::string>(weights[k]).c_str();

            //set predecessor data
            pugi::xml_node predecessorRoot = currentNeuron.append_child("Predecessors");
            set<uint> predecessors = iter->second->getPredecessors();
            for(set<uint>::iterator predIter = predecessors.begin(); predIter != predecessors.end(); predIter++)
                predecessorRoot.append_child("Predecessor").append_attribute("ID") = *predIter;

        }
    }
}

map<uint, NeuronInfo> NeuralNetwork::getMapStructure(){
    map<uint, NeuronInfo> output;
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        NeuronInfo currNeuronInfo;

        if(iter->second->getNeuronType() == LEAF)
            currNeuronInfo.neuronType = LEAF;
        else{
            map<uint, Neuron*>::const_iterator outputCheckIterator = mOutput.find(iter->first);
            if(outputCheckIterator != mOutput.end())
                currNeuronInfo.neuronType = OUTPUT;
            else currNeuronInfo.neuronType = NONLEAF;
            
            currNeuronInfo.activationFunction = iter->second->getActivationFunction();
            currNeuronInfo.predecessors = iter->second->getPredecessors();
            currNeuronInfo.weights = iter->second->getWeights();
        }
        output[iter->first] = currNeuronInfo;
    }

    return output;
}

map<uint, vector<double>> NeuralNetwork::getWeights(){
    map<uint, vector<double>> output;
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++)
        if(iter->second->getNeuronType() == NONLEAF)
            output[iter->first] = iter->second->getWeights();
    
    return output;
}

void NeuralNetwork::setWeights(map<uint, vector<double>> _weights){
    for(map<uint, vector<double>>::iterator iter = _weights.begin(); iter != _weights.end(); iter++)
        mNeuronCache[iter->first]->setWeights(iter->second);
}

void NeuralNetwork::serialize(int*& _nodes, int*& _format, double*& _weights, int& _formatSize, int& _weightSize){
    _nodes = new int[3];
    _nodes[0] = 0;
    _nodes[1] = 0;
    _nodes[2] = 0;

    _formatSize = _weightSize = 0;

    //calculate buffer sizes required
    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second->getNeuronType() == LEAF)
            _nodes[0]++;
        else{
            _formatSize+=3;

            map<uint, Neuron*>::const_iterator outputCheckIterator = mOutput.find(iter->first);
            if(outputCheckIterator != mOutput.end())
                _nodes[2]++;
            else _nodes[1]++;

            _formatSize += iter->second->getPredecessors().size();
            _weightSize += iter->second->getWeights().size();
        }
    }

    _format = new int[_formatSize];
    _weights = new double[_weightSize];

    int currentFormatPos = 0;
    int currentWeightPos = 0;

    for(map<uint, Neuron*>::iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        if(iter->second->getNeuronType() != LEAF){
            _format[currentFormatPos++] = iter->first;

            map<uint, Neuron*>::const_iterator outputCheckIterator = mOutput.find(iter->first);
            if(outputCheckIterator != mOutput.end())
                _format[currentFormatPos++] = 1;
            else _format[currentFormatPos++] = 0;

            _format[currentFormatPos++] = iter->second->getPredecessors().size();

            set<uint> currPredecessors = iter->second->getPredecessors();

            for(set<uint>::iterator formatIter = currPredecessors.begin(); formatIter != currPredecessors.end(); formatIter++)
                _format[currentFormatPos++] = *formatIter;

            for(uint k = 0; k < iter->second->getWeights().size(); ++k)
                _weights[currentWeightPos++] = iter->second->getWeights()[k];
        }
    }
}

NeuralNetwork::NeuralNetwork(int* _nodes, int* _format, double* _weights, int _formatSize, int _weightSize){
    int input = _nodes[0], hidden = _nodes[1], output = _nodes[2];

    for(uint k = 1; k <= input; ++k)
        mNeuronCache[k] = new LeafNeuron(&mNeuronCache, vector<double>());

    int currentFormatPos = 0;
    int currentWeightPos = 0;
    
    for(uint k = 0; k < hidden + output; ++k){
        int nodeid = _format[currentFormatPos++];
        int nodetype = _format[currentFormatPos++];
        int numPredecessors = _format[currentFormatPos++];

        set<uint> predecessors;
        vector<double> weights;

        for(uint i = 0; i < numPredecessors; ++i){
            predecessors.insert(_format[currentFormatPos++]);
            weights.push_back(_weights[currentWeightPos++]);
        }
        //+1 for bias
        weights.push_back(_weights[currentWeightPos++]);

        Neuron* currNeuron = new NonLeafNeuron(&mNeuronCache, weights, SIGMOID);
        currNeuron->setInput(predecessors, false);
        mNeuronCache[nodeid] = currNeuron;

        if(nodetype == 1)
            mOutput[nodeid] = currNeuron;
    }

    delete [] _nodes;
    delete [] _format;
    delete [] _weights;
}

bool NeuralNetwork::operator==(const NeuralNetwork& _other)const{
    if(mNeuronCache.size() != _other.mNeuronCache.size() || mOutput.size() != _other.mOutput.size())
        return false;

    for(map<uint, Neuron*>::const_iterator iter = mNeuronCache.begin(); iter != mNeuronCache.end(); iter++){
        map<uint, Neuron*>::const_iterator otherIter = _other.mNeuronCache.find(iter->first);

        if(otherIter == _other.mNeuronCache.end())
            return false;

        if(iter->second->getNeuronType() == NONLEAF){
            if(iter->second->getPredecessors() != otherIter->second->getPredecessors())
                return false;

            if(iter->second->getWeights() != otherIter->second->getWeights())
                return false;
        }
    }

    return true;
}
void NeuralNetwork::setTeamID(uint _id){
    mTeamID = _id;
}

uint NeuralNetwork::getTeamID(){
    return mTeamID;
}
