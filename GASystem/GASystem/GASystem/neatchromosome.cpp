#include "neatchromosome.h"

using namespace std;

NeatChromosome::NeatChromosome(){
}

NeatChromosome::NeatChromosome(const NeatChromosome& other){
    mGenotype = other.mGenotype;
    mNodes = other.mNodes;
    mInnovation = other.mInnovation;
    mMutationList = other.mMutationList;
    mConLookup = other.mConLookup;
}

NeatChromosome::~NeatChromosome(){

}

NeatChromosome::NeatChromosome(const vector<vector<NeatGene>> _genotype, const vector<vector<NeatNode>> _nodes){
    mGenotype = _genotype;
    mNodes = _nodes;

    for(uint k = 0; k < mGenotype.size(); ++k){
        mConLookup.push_back(map<pair<long, long>, bool>());
        for(uint i = 0; i < mGenotype[k].size(); ++i){
            mConLookup[k][make_pair(mGenotype[k][i].inID, mGenotype[k][i].outID)] = true;
        }
    }
}

void NeatChromosome::setGlobalDat(long* _innovation, map<pair<long, long>, long>* _mutList){
    mInnovation = _innovation;
    mMutationList = _mutList;
}

NeatChromosome& NeatChromosome::operator = (const NeatChromosome& other){
    mGenotype = other.mGenotype;
    mNodes = other.mNodes;
    mInnovation = other.mInnovation;
    mMutationList = other.mMutationList;
    mConLookup = other.mConLookup;

    return *this;
}

bool NeatChromosome::initialize(pugi::xml_node* _root){
    for(pugi::xml_node currNetwork = _root->first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
        vector<NeatGene> currNNGenes;
        vector<NeatNode> currNNNodes;
        
        constructNeatANN(&currNetwork, currNNGenes, currNNNodes);

        mGenotype.push_back(currNNGenes);
        mNodes.push_back(currNNNodes);
    }
    return true;
}

bool NeatChromosome::constructNeatANN(pugi::xml_node* _nnRootNode, vector<NeatGene>& _currNNGenes, vector<NeatNode>& _currNNNodes){
    _currNNGenes.clear();
    _currNNNodes.clear();
	long innovNum = 0;
    
    uint teamID;
	if(_nnRootNode->attribute("TeamID").empty()){
        cerr << "No TeamID defined, defaulting team to 0" << endl;
		teamID = 0;
    }
	else teamID = atoi(_nnRootNode->attribute("TeamID").value());

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(-1, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    int nnPos = 0;

    for(pugi::xml_node node = _nnRootNode->first_child(); node; node = node.next_sibling()){
        mConLookup.push_back(map<pair<long, long>, bool>());
        if(node.attribute("ID").empty()){
            cerr << "Error: node does not have ID" << endl;
            return false;
        }

        if(node.attribute("Type").empty()){
            cerr << "Error: node does not have a Type" << endl;
            return false;
        }

        NeatNode currNode;

        currNode.ID = atoi(node.attribute("ID").value());
        currNode.teamID = teamID;

        //if input type, create leaf neuron, otherwise create a nonleaf neuron
        if(strcmp(node.attribute("Type").value(), "Input") == 0)
            currNode.geneType = LEAF;
        else{
            vector<double> weights;
            set<uint> predecessors;

            //checks
            if(node.attribute("ActivationFunction").empty()){
                cerr << "Error: Non input nodes must have an Activation Function" << endl;
                return false;
            }

            if(node.child("Predecessors").empty()){
                cerr << "Error: Non input nodes must have Predecessors" << endl;
                return false;
            }

            //assign activation function
            if(strcmp(node.attribute("ActivationFunction").value(), "Sigmoid") == 0)
                currNode.activationFunc = SIGMOID;
            else{
                    cerr << "Error: unable to understand the activation function of neuron " << currNode.ID << ", defaulting to sigmoid" << endl;
                    currNode.activationFunc = SIGMOID;
            }
            
            //assign predecessors
            pugi::xml_node predecessorsRoot = node.child("Predecessors");
            for(pugi::xml_node predecessorNode = predecessorsRoot.first_child(); predecessorNode; predecessorNode = predecessorNode.next_sibling()){
                if(predecessorNode.attribute("ID").empty()){
                    cerr << "Error: Predecessors must have an ID" << endl;
                    return false;
                }

                int predecID = atoi(predecessorNode.attribute("ID").value());
                NeatGene gene;
                gene.enabled = true;
                gene.inID = predecID;
                gene.outID = currNode.ID;
                gene.weight = gen();
                gene.innovation = ++innovNum;

                _currNNGenes.push_back(gene);

                mConLookup[nnPos][make_pair(predecID, currNode.ID)] = true;
            }

            currNode.geneType = strcmp(node.attribute("Type").value(), "Output") == 0 ? OUTPUT : NONLEAF;
        }

        currNode.biasWeight = gen();
        _currNNNodes.push_back(currNode);
        nnPos++;
    }

    if(innovNum > *mInnovation)
        *mInnovation = innovNum;

    return true;
}

void NeatChromosome::mutate(string _mutationType, map<string, double>& _parameters){
    if(_parameters.find("Deviation") == _parameters.end()){
        cout << "Error: no mutation prob" << endl;
        return;
    }

    if(_parameters.find("MutationProbability") == _parameters.end()){
        cout << "Error: no standard deviation specified for mutation" << endl;
        return;
    }

    if(_parameters.find("WeightMutationProbability") == _parameters.end()){
        cout << "Error: no standard deviation specified for mutation" << endl;
        return;
    }

    double mutProb = _parameters["MutationProbability"];
    double mutAddProb = _parameters["MutationAddProbability"];
    double weightMutProb = _parameters["WeightMutationProbability"];
    double deviation = _parameters["Deviation"];

    mutateWeight(weightMutProb, deviation);
    mutateAddConnection(mutProb);
    mutateAddNode(mutAddProb);
}

void NeatChromosome::mutateAddNode(double _mutationProb){
    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    for(uint k = 0; k < mGenotype.size(); ++k){
        if(gen() < _mutationProb){
            boost::mt19937 rngCon(rand());
            boost::uniform_int<> distCon(0, mGenotype[k].size() - 1);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genCon(rngCon, distCon);

            boost::uniform_real<double> distBias(-1, 1);
            boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genBias(rngCon, distBias);

            int pos = genCon();

            mGenotype[k][pos].enabled = false;

            NeatNode newnode;
            newnode.ID = mNodes[k][mNodes[k].size() - 1].ID + 1;
            newnode.teamID = mNodes[k][0].teamID;
            newnode.geneType = NONLEAF;
            newnode.biasWeight = genBias();
            newnode.activationFunc = SIGMOID;
            mNodes[k].push_back(newnode);

            NeatGene predGene;
            predGene.inID = mGenotype[k][pos].inID;
            predGene.outID = newnode.ID;
            predGene.weight = 1;
            predGene.enabled = true;
            pair<long, long> predk = make_pair(predGene.inID, predGene.outID);

            if(mMutationList->find(predk) == mMutationList->end()){
                predGene.innovation = ++*mInnovation;
                mMutationList->insert(make_pair(predk, *mInnovation));
            }
            else predGene.innovation = (*mMutationList)[predk];

            NeatGene sucGene;
            sucGene.inID = newnode.ID;
            sucGene.outID = mGenotype[k][pos].outID;
            sucGene.weight = mGenotype[k][pos].weight;
            sucGene.enabled = true;
            pair<long, long> suck = make_pair(sucGene.inID, sucGene.outID);

            if(mMutationList->find(suck) == mMutationList->end()){
                sucGene.innovation = ++*mInnovation;
                (*mMutationList)[suck] = *mInnovation;
            }
            else sucGene.innovation = (*mMutationList)[suck];

            mGenotype[k].push_back(sucGene);
            mGenotype[k].push_back(predGene);
            mConLookup[k][make_pair(sucGene.inID, sucGene.outID)] = true;
            mConLookup[k][make_pair(predGene.inID, predGene.outID)] = true;
        }
    }
}

void NeatChromosome::mutateAddConnection(double _mutationProb){
    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    for(uint k = 0; k < mNodes.size(); ++k){
        for(uint i = 0; i < mNodes[k].size(); ++i){
            for(uint l = 0; l < mNodes[k].size(); ++l){
                if(i == l || mNodes[k][i].geneType == OUTPUT || mNodes[k][l].geneType == LEAF)
                    continue;

                pair<long, long> con = make_pair(mNodes[k][i].ID, mNodes[k][l].ID);
                pair<long, long> recipCon = make_pair(mNodes[k][l].ID, mNodes[k][i].ID);
                if(mConLookup[k].find(con) != mConLookup[k].end() || mConLookup[k].find(recipCon) != mConLookup[k].end())
                    continue;

                if(gen() < _mutationProb){
                    if(!checkGeneLoops(k, mNodes[k][i].ID, mNodes[k][l].ID))
                        continue;

                    NeatGene gene;
                    gene.inID = mNodes[k][i].ID;
                    gene.outID = mNodes[k][l].ID;
                    gene.weight = mGenotype[k][i].weight;
                    gene.enabled = true;

                    if(mMutationList->find(con) == mMutationList->end()){
                        gene.innovation = ++*mInnovation;
                        (*mMutationList)[con] = *mInnovation;
                    }
                    else gene.innovation = (*mMutationList)[con];

                    mGenotype[k].push_back(gene);
                    mConLookup[k][con] = true;
                }
            }
        }
    }
}

bool NeatChromosome::checkGeneLoops(uint _ann, uint _predec, uint _target){
    for(uint k = 0; k < mGenotype[_ann].size(); ++k){
        if(mGenotype[_ann][k].outID == _predec){
            if(mGenotype[_ann][k].inID == _target)
                return false;
            
            if(!checkGeneLoops(_ann, mGenotype[_ann][k].inID, _target))
                return false;
        }
    }

    return true;
}

void NeatChromosome::mutateWeight(double _mutationProb, double _deviation){
    boost::mt19937 mRNGMutation(rand());
    boost::normal_distribution<> mutationDist(0, _deviation);
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > genMutation(mRNGMutation, mutationDist);

    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);


    for(uint k = 0; k < mGenotype.size(); k++){
        if(gen() < _mutationProb){
            for(uint i = 0; i < mGenotype[k].size(); ++i){
                mGenotype[k][i].weight += genMutation();
                if(mGenotype[k][i].weight > 1)
                    mGenotype[k][i].weight = 1;
                if(mGenotype[k][i].weight < -1)
                    mGenotype[k][i].weight = -1;
            }
        }
    }
}

vector<map<uint, vector<double>>> NeatChromosome::getWeightData(){
    cerr << "Error: Get weights is not viable for a NEAT chromosome" << endl;

    return vector<map<uint, vector<double>>>();
}

vector<map<uint, NeuronInfo>> NeatChromosome::getFullStructureData(){
    vector<map<uint, NeuronInfo>> chromStructDat;
    vector<map<uint, vector<uint>>> predecMap;
    vector<map<uint, vector<double>>> weightMap;


    for(uint k = 0; k < mGenotype.size(); ++k){
        map<uint, vector<uint>> currNNpMap;
        map<uint, vector<double>> currNNwMap;
        for(uint i = 0; i < mGenotype[k].size(); ++i){
            if(!mGenotype[k][i].enabled)
                continue;

            int nnID = mGenotype[k][i].outID;
            int predID = mGenotype[k][i].inID;
            double weight = mGenotype[k][i].weight;

            if(currNNpMap.find(nnID) == currNNpMap.end()){
                vector<uint> pred;
                vector<double> predWeights;
                pred.push_back(predID);
                predWeights.push_back(weight);
                currNNpMap[nnID] = pred;
                currNNwMap[nnID] = predWeights;
            }
            else{
                currNNpMap[nnID].push_back(predID);
                currNNwMap[nnID].push_back(weight);
            }
        }
        predecMap.push_back(currNNpMap);
        weightMap.push_back(currNNwMap);
    }

    for(uint k = 0; k < mNodes.size(); ++k){
        map<uint, NeuronInfo> currNetwork;
        for(uint i = 0; i < mNodes[k].size(); ++i){
            NeuronInfo node;
            
            node.neuronType = mNodes[k][i].geneType;
            node.activationFunction = mNodes[k][i].activationFunc;
            node.predecessors = predecMap[k][mNodes[k][i].ID];
            node.weights = weightMap[k][mNodes[k][i].ID];
            node.weights.push_back(mNodes[k][i].biasWeight);
            node.teamID = mNodes[k][i].teamID;

            currNetwork[mNodes[k][i].ID] = node;
        }
        chromStructDat.push_back(currNetwork);
    }

    return chromStructDat;
}

void NeatChromosome::setWeights(vector<map<uint, vector<double>>>& _weights){
    cerr << "Error: Set weights is not viable for a NEAT chromosome" << endl;
}

void NeatChromosome::setStructure(vector<map<uint, NeuronInfo>>& structure){
    long innovNum = 0;
    mGenotype.clear();
    mNodes.clear();
    mConLookup.clear();

    for(uint k = 0; k < structure.size(); ++k){
        vector<NeatGene> currNetGenes;
        vector<NeatNode> currNetNodes;
        mConLookup.push_back(map<pair<long, long>, bool>());

        for(map<uint, NeuronInfo>::iterator iter = structure[k].begin(); iter != structure[k].end(); ++iter){
            NeatNode node;

            node.ID = iter->first;
            node.activationFunc = iter->second.activationFunction;
            node.teamID = iter->second.teamID;
            node.geneType = iter->second.neuronType;
            node.biasWeight = iter->second.weights[iter->second.weights.size() - 1];

            vector<uint> predecessors = iter->second.predecessors;
            uint pos = 0;
            for(uint k = 0; k < predecessors.size(); ++k){
                NeatGene gene;
                gene.enabled = true;
                gene.inID = predecessors[k];
                gene.weight = iter->second.weights[pos++];
                gene.outID = node.ID;
                gene.innovation = ++innovNum;

                currNetGenes.push_back(gene);
                mConLookup[k][make_pair(gene.inID, gene.outID)] = true;
            }
            currNetNodes.push_back(node);
        }

        mGenotype.push_back(currNetGenes);
        mNodes.push_back(currNetNodes);
    }

    if(innovNum > *mInnovation)
        *mInnovation = innovNum;
}

Chromosome* NeatChromosome::clone(){
    return new NeatChromosome(*this);
}

bool NeatChromosome::addDelta(vector<map<uint, vector<double>>> _weights){
    cerr << "Error: Delta coding is not viable for NEAT chromosome" << endl;

    return false;
}

void NeatChromosome::reInitialize(){
    boost::mt19937 rng(rand());
    boost::uniform_real<double> dist(-1, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    for(uint k = 0; k < mGenotype.size(); ++k)
        for(uint i = 0; i < mGenotype[k].size(); ++i)
            mGenotype[k][i].weight = gen();

    for(uint k = 0; k < mNodes.size(); ++k)
        for(uint i = 0; i < mNodes[k].size(); ++i)
            mNodes[k][i].biasWeight = gen();
}

vector<NeuralNetwork> NeatChromosome::getNets(){
    vector<NeuralNetwork> nets;
    vector<map<uint, NeuronInfo>> structDat = getFullStructureData();

    for(uint k = 0; k < structDat.size(); ++k){
        nets.push_back(NeuralNetwork(structDat[k]));
    }

    return nets;
}

double NeatChromosome::calcCompDistance(NeatChromosome* _other){
    if(mGenotype.size() != _other->mGenotype.size()){
        cerr << "Error: Genotypes do not match, cannot calculate compatibility distance" << endl;
        return 0;
    }

    double c1 = 1;
    double c2 = 0.4;
    
    double totMatching = 0;
    double totWeightDiscrep = 0;
    double totDisjoint = 0;
    int s1 = 0, s2 = 0;

    for(uint k = 0; k < mGenotype.size(); ++k){
        double matching = 0;
        double total = (double)(mGenotype[k].size() + _other->mGenotype[k].size());
        s1 += mGenotype[k].size();
        s2 += _other->mGenotype[k].size();

        for(uint l = 0; l < mGenotype[k].size(); ++l){
            for(uint i = 0; i < _other->mGenotype[k].size(); ++i){
                if(mGenotype[k][l].innovation == _other->mGenotype[k][i].innovation){
                    matching += 1;
                    totWeightDiscrep += fabs(mGenotype[k][l].weight - _other->mGenotype[k][i].weight);
                    break;
                }
            }
        }
        
        totDisjoint = total - 2*matching;
        totMatching += matching;
    }

    int maxs = s1 > s2 ? s1 : s2;

    totWeightDiscrep /= totMatching;

    double val = c1 * totDisjoint/maxs + c2 * totWeightDiscrep;

    return val;
}

Chromosome* NeatChromosome::produceOffspring(Chromosome* _other){
    boost::mt19937 rng(rand());
    boost::uniform_int<> dist(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_int<>> gen(rng, dist);

    if(mGenotype.size() != dynamic_cast<NeatChromosome*>(_other)->mGenotype.size()){
        cerr << "Error: Genotypes do not match, cannot reproduce" << endl;
        return NULL;
    }

    NeatChromosome* higher;
    NeatChromosome* lower;

    if(mFitness == _other->fitness()){
        int choice = gen();
        higher = choice == 0 > _other->fitness() ? this : dynamic_cast<NeatChromosome*>(_other);
        lower = choice == 1 < _other->fitness() ? this : dynamic_cast<NeatChromosome*>(_other);
    }
    else{
        higher = mFitness > _other->fitness() ? this : dynamic_cast<NeatChromosome*>(_other);
        lower = mFitness < _other->fitness() ? this : dynamic_cast<NeatChromosome*>(_other);
    }

    vector<vector<NeatGene>> newGenotype;
    vector<vector<NeatNode>> newNodes = higher->mNodes;

    for(uint k = 0; k < newNodes.size(); ++k){
        for(uint l = 0; l < newNodes[k].size(); ++l){
            for(uint i = 0; i < lower->mNodes[k].size(); ++i){
                if(newNodes[k][l].ID == lower->mNodes[k][i].ID){
                    if(gen() == 0)
                        newNodes[k][l].biasWeight = lower->mNodes[k][i].biasWeight;
                }
            }
        }
    }

    for(uint k = 0; k < higher->mGenotype.size(); ++k){
        vector<NeatGene> currNet;
        for(uint l = 0; l < higher->mGenotype[k].size(); ++l){
            NeatGene currGene = higher->mGenotype[k][l];

            int foundPos = -1;
            for(uint i = 0; i < lower->mGenotype[k].size(); ++i){
                if(higher->mGenotype[k][l].innovation == lower->mGenotype[k][i].innovation){
                    foundPos = i;
                    break;
                }
            }

            if(foundPos > -1)
                currGene.weight = gen() == 0 ? lower->mGenotype[k][foundPos].weight : currGene.weight;
            
            currNet.push_back(currGene);
        }
        newGenotype.push_back(currNet);
    }

    Chromosome* offspring = new NeatChromosome(newGenotype, newNodes);
    dynamic_cast<NeatChromosome*>(offspring)->setGlobalDat(mInnovation, mMutationList);

    return offspring;
}