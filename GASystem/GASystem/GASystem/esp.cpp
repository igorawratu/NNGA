#include "esp.h"

ESP::ESP(ESPParameters _parameters){
    mParameters = _parameters;

    setupSubpopulationStructure();
}

ESP::~ESP(){
    for(uint k = 0; k < mSubpopulations.size(); ++k){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            delete iter->second.first;
        }
    }
}

Solution ESP::train(SimulationContainer* _simulationContainer, string _outputFileName){
    //evaluate
    mBestFitness = mBestRealFitness = -1;
    evaluateFitness(_simulationContainer);
    for(uint k = 0; k < mParameters.maxGenerations; ++k){
        time_t t = time(0);

        cout << "Generation " << k << endl;
        if(mStagnationCounter >= mParameters.stagnationThreshold){
            cout << "Population stagnated, generating Delta Codes" << endl;
            runDeltaCodes(_simulationContainer);
            mStagnationCounter = 0;
        }
        else{
            cout << "Generating and mutating offspring" << endl;
            for(uint i = 0; i < mSubpopulations.size(); ++i){
                for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
                    if(iter->second.second != 0)
                        iter->second.first->generateOffspring();
                }
            }
        }

        cout << "Evaluating Fitness" << endl;
        evaluateFitness(_simulationContainer);

        cout << "Best Fitness: " << mBestFitness << " - " << mBestRealFitness << endl;

        for(uint i = 0; i < mSubpopulations.size(); ++i){
            for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
                if(iter->second.second != 0)
                    iter->second.first->nextGeneration();
            }
        }

        cout << "Time taken for this generation : " << time(0) - t << endl;

        if(mBestRealFitness <= mParameters.fitnessEpsilonThreshold)
            return mBestSolution;

        if((k + 1) % 50 == 0 || k == 0){
            ofstream outFile;
            outFile.open(_outputFileName.c_str(), ofstream::app);
            
            outFile << (k == 0? k : k + 1) << ": " << mBestSolution.fitness() << " " << mBestSolution.realFitness() << endl;
            outFile.close();
        }
    }

    return mBestSolution;
}

void ESP::evaluateFitness(SimulationContainer* _simulationContainer){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    bool improved = false;
    while(createNeuralNetworkPrimitives(neuralNetPrimitives)){
        vector<NeuralNetwork> neuralNets;
        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[k].setStructure(neuralNetPrimitives[k].first, neuralNetPrimitives[k].second);
        }
        Solution solution(neuralNets);
        _simulationContainer->runFullSimulation(&solution);
        _simulationContainer->resetSimulation();
        
        if(solution.realFitness() <= mParameters.fitnessEpsilonThreshold || mBestFitness == -1 || (mBestFitness > solution.fitness() && mBestRealFitness > mParameters.fitnessEpsilonThreshold)){
            mBestFitness = solution.fitness();
            mBestRealFitness = solution.realFitness();
            mBestSolution = solution;
            improved = true;
        }

        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[k].first.begin(); iter != neuralNetPrimitives[k].first.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
                else
                    mSubpopulations[k][iter->first].first->setChromosomeFitness(iter->second, solution.fitness(), solution.realFitness());
            }
        }
    }

    mStagnationCounter = improved ? 0 : mStagnationCounter + 1;
}

bool ESP::createNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> out;
    bool completed = false;

    for(uint k = 0; k < mSubpopulations.size(); ++k){
        map<uint, Neuron*> output;
        map<uint, Neuron*> neuronCache;

        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            if(iter->second.second == 0)
                neuronCache[iter->first] = new LeafNeuron(NULL, vector<double>());
            else{
                Chromosome* chrom = iter->second.first->getUnevaluatedChromosome();
                if(chrom == NULL){
                    completed = true;
                    break;
                }

                neuronCache[iter->first] = dynamic_cast<ESPChromosome*>(chrom)->getNeuron();
                if(iter->second.second == 2)
                    output[iter->first] = neuronCache[iter->first];
            }
        }
        if(completed){
            for(map<uint, Neuron*>::iterator iter = neuronCache.begin(); iter != neuronCache.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
            }
            return false;
        }   
        out.push_back(make_pair(neuronCache, output));
    }
    _output = out;

    return true;
}

bool ESP::createDeltaNeuralNetworkPrimitives(vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>>& _output){
    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> out;
    bool completed = false;

    for(uint k = 0; k < mSubpopulations.size(); ++k){
        map<uint, Neuron*> output;
        map<uint, Neuron*> neuronCache;

        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[k].begin(); iter != mSubpopulations[k].end(); ++iter){
            if(iter->second.second == 0)
                neuronCache[iter->first] = new LeafNeuron(NULL, vector<double>());
            else{
                Chromosome* chrom = iter->second.first->getUnevaluatedDeltaCode();
                if(chrom == NULL){
                    completed = true;
                    break;
                }

                neuronCache[iter->first] = dynamic_cast<ESPChromosome*>(chrom)->getNeuron();
                if(iter->second.second == 2)
                    output[iter->first] = neuronCache[iter->first];
            }
        }
        if(completed){
            for(map<uint, Neuron*>::iterator iter = neuronCache.begin(); iter != neuronCache.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
            }
            return false;
        }   
        out.push_back(make_pair(neuronCache, output));
    }
    _output = out;

    return true;
}

void ESP::setupSubpopulationStructure(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(mParameters.nnFormatFilename.c_str());
    if(!result){
        cerr << "Error: unable to parse the file " << mParameters.nnFormatFilename << endl;
        return;
    }

    pugi::xml_node root = doc.first_child();

    for(pugi::xml_node currNetwork = root.first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
        map<uint, pair<ESPSubPopulation*, uint>> currNetworkSubpopulations;
        for(pugi::xml_node node = currNetwork.first_child(); node; node = node.next_sibling()){
            if(node.attribute("ID").empty()){
                cerr << "Error: node does not have ID" << endl;
                return;
            }
            
            uint neuronID = atoi(node.attribute("ID").value());
            
            if(node.attribute("Type").empty()){
                cerr << "Error: node does not have a Type" << endl;
                return;
            }

            uint neuronType;
            
            if(strcmp(node.attribute("Type").value(), "Input") == 0)
                neuronType = 0;
            else if(strcmp(node.attribute("Type").value(), "Hidden") == 0)
                neuronType = 1;
            else if(strcmp(node.attribute("Type").value(), "Output") == 0)
                neuronType = 2;

            ESPSubPopulation* subpop = neuronType == 0 ? NULL : new ESPSubPopulation(mParameters, &node);
            currNetworkSubpopulations[neuronID] = make_pair(subpop, neuronType);
        }

        mSubpopulations.push_back(currNetworkSubpopulations);
    }

}

void ESP::runDeltaCodes(SimulationContainer* _simulationContainer){
    for(uint i = 0; i < mSubpopulations.size(); ++i){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
            if(iter->second.second != 0)
                iter->second.first->generateDeltaCodes();
        }
    }

    vector<pair<map<uint, Neuron*>, map<uint, Neuron*>>> neuralNetPrimitives;
    while(createDeltaNeuralNetworkPrimitives(neuralNetPrimitives)){
        vector<NeuralNetwork> neuralNets;
        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            neuralNets.push_back(NeuralNetwork());
            neuralNets[k].setStructure(neuralNetPrimitives[k].first, neuralNetPrimitives[k].second);
        }
        Solution solution(neuralNets);
        _simulationContainer->runFullSimulation(&solution);
        _simulationContainer->resetSimulation();

        for(uint k = 0; k < neuralNetPrimitives.size(); ++k){
            for(map<uint, Neuron*>::iterator iter = neuralNetPrimitives[k].first.begin(); iter != neuralNetPrimitives[k].first.end(); ++iter){
                if(iter->second->getNeuronType() == LEAF)
                    delete iter->second;
                else{
                    mSubpopulations[k][iter->first].first->setDeltaCodeFitness(iter->second, solution.fitness(), solution.realFitness());
                }
            }
        }
    }

    for(uint i = 0; i < mSubpopulations.size(); ++i){
        for(map<uint, pair<ESPSubPopulation*, uint>>::iterator iter = mSubpopulations[i].begin(); iter != mSubpopulations[i].end(); ++iter){
            if(iter->second.second != 0)
                iter->second.first->integrateDeltaCodes();
        }
    }
}