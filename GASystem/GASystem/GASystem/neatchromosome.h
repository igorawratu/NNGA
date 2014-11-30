#ifndef NEATCHROMOSOME_H
#define NEATCHROMOSOME_H

#include "chromosome.h"
#include "neuralnetwork.h"

struct NeatGene{
    uint inID;
    uint outID;
    double weight;
    bool enabled;
    long innovation;
};

struct NeatNode{
    uint ID;
    uint teamID;
    NeuronType geneType;
    double biasWeight;
    ActivationFunction activationFunc;
};

class NeatChromosome : public Chromosome
{
public:
    NeatChromosome();
    NeatChromosome(const vector<vector<NeatGene>> _genotype, const vector<vector<NeatNode>> _nodes);
    NeatChromosome(const NeatChromosome& other);
    virtual ~NeatChromosome();
    NeatChromosome& operator = (const NeatChromosome& other);

    void setGlobalDat(long* _innovation, map<pair<long, long>, long>* _mutList);
    virtual bool initialize(pugi::xml_node* _root);
    virtual void mutate(string _mutationType, map<string, double>& _parameters);
    virtual vector<map<uint, vector<double>>> getWeightData();
    virtual vector<map<uint, NeuronInfo>> getFullStructureData();
    virtual void setWeights(vector<map<uint, vector<double>>>& _weights);
    virtual void setStructure(vector<map<uint, NeuronInfo>>& structure);
    virtual Chromosome* clone();
    virtual bool addDelta(vector<map<uint, vector<double>>> _weights);
    virtual void reInitialize();
    vector<NeuralNetwork> getNets();
    double calcCompDistance(NeatChromosome* _other);
    Chromosome* produceOffspring(Chromosome* _other);

public:
    int species;

private:
    bool constructNeatANN(pugi::xml_node* _nnRootNode, vector<NeatGene>& _currNNGenes, vector<NeatNode>& _currNNNodes);
    void mutateAddNode(double _mutationProb);
    void mutateAddConnection(double _mutationProb);
    void mutateWeight(double _mutationProb, double _deviation);
    bool checkGeneLoops(uint _ann, uint _predec, uint _target);

private:
    vector<vector<NeatGene>> mGenotype;
    vector<vector<NeatNode>> mNodes;
    long* mInnovation;
    map<pair<long, long>, long>* mMutationList;
    vector<map<pair<long, long>, bool>> mConLookup;
};

#endif