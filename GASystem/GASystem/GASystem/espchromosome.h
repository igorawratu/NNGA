#ifndef ESPCHROMOSOME_H
#define ESPCHROMOSOME_H

#include "chromosome.h"
#include "nonleafneuron.h"
#include "mutation.h"
#include "mutationfactory.h"
#include "common.h"

#include <map>
#include <vector>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

class ESPChromosome : public Chromosome
{
public:
    ESPChromosome();
    ESPChromosome(const ESPChromosome& other);
    virtual ~ESPChromosome();
    ESPChromosome& operator = (const ESPChromosome& other);

    virtual bool initialize(pugi::xml_node* _root);
    virtual void mutate(string _mutationType, map<string, double>& _parameters);
    virtual vector<map<uint, vector<double>>> getWeightData();
    virtual vector<map<uint, NeuronInfo>> getFullStructureData();
    virtual void setWeights(vector<map<uint, vector<double>>>& _weights);
    virtual void setStructure(vector<map<uint, NeuronInfo>>& structure);
    virtual Chromosome* clone();

private:
    NonLeafNeuron* mNeuron;

};

#endif