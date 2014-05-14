#include <iostream>
#include <map>
#include <string>
#include <typeinfo>
#include <vector>
#include <mpi.h>
#include <fstream>

#include "common.h"
#include "neuralnetwork.h"
#include "pugixml.hpp"
#include "nnchromosome.h"
#include "solution.h"
#include "gaussianmutation.h"
#include "common.h"
#include "multipointcrossover.h"
#include "quadraticrankselection.h"
#include "selectionfactory.h"
#include "mutationfactory.h"
#include "crossoverfactory.h"
#include "standardga.h"
#include "dummyfitness.h"
#include "dummysimulation.h"
#include "simulationcontainer.h"
#include "gaengine.h"
#include "graphicsengine.h"
#include "prototypesimulation.h"
#include "waypointfitness.h"
#include "collisionfitness.h"
#include "bridgesimulation.h"
#include "finishlinefitness.h"
#include "corneringsim.h"
#include "carcrashsimulation.h"
#include "carracesimulation.h"
#include "warrobotsimulation.h"
#include "mouseescapesimulation.h"
#include "mousescattersimulation.h"
#include "sfobstaclesimulation.h"
#include "sfturnbacksimulation.h"
#include "sfobstaclefieldsimulation.h"
#include "esp.h"
#include "polebalancingsimulation.h"
#include <boost/lexical_cast.hpp>
#include "slave.h"
#include <boost/tuple/tuple.hpp>

#define TRAIN

using namespace std;

enum GAType{TYPE_STANDARD, TYPE_ESP};

typedef boost::tuples::tuple<Simulation*, string, string> SimInfo;

void testCrossoverOp(){
    srand(time(0));
    Crossover* crossoverAlgorithm = CrossoverFactory::instance().create("LX");
    vector<Chromosome*> parents;
    map<string, double> params;
    params["CrossoverProbability"] = 1;

    //p1
    xmldoc p1doc;
    p1doc.load_file("neuralxmls/crossovertest/p1.xml");
    pugi::xml_node root = p1doc.first_child();

    NNChromosome* p1 = new NNChromosome();
    p1->initialize(&root);

    //p2
    xmldoc p2doc;
    p2doc.load_file("neuralxmls/crossovertest/p2.xml");
    root = p2doc.first_child();

    NNChromosome* p2 = new NNChromosome();
    p2->initialize(&root);
    
    //p3
    xmldoc p3doc;
    p3doc.load_file("neuralxmls/crossovertest/p3.xml");
    root = p3doc.first_child();

    NNChromosome* p3 = new NNChromosome();
    p3->initialize(&root);

    parents.push_back(p3);
    parents.push_back(p2);
    parents.push_back(p1);
    
    vector<Chromosome*> offspring = crossoverAlgorithm->execute(parents, 2000, params, SelectionFactory::instance().create("RandomSelection"));

    ofstream outputFile, y;
    outputFile.open("neuralxmls/crossovertest/offspring.txt");
    y.open("neuralxmls/crossovertest/y.txt");

    for(uint k = 0; k < offspring.size(); ++k){
        vector<map<uint, vector<double>>> childWeights = offspring[k]->getWeightData();
        for(uint k = 0; k < childWeights.size(); k++){
            for(map<uint, vector<double>>::iterator iter = childWeights[k].begin(); iter != childWeights[k].end(); iter++){
                outputFile << iter->second[0] << endl;       
                y << iter->second[1] << endl; 
            }
        }
    }
    outputFile.close();
    y.close();

}

StandardGAParameters getGAParameters(int _popsize, int _maxgen, string _nnFormatFile, double _stag, double _epsilon){
    StandardGAParameters params;
    params.populationSize = _popsize;
    params.maxGenerations = _maxgen;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = _stag;
    params.fitnessEpsilonThreshold = _epsilon;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "LX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = _popsize/10;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    return params;
}

ESPParameters getESPParameters(int _popsize, int _maxgen, string _nnFormatFile, double _stag, double _epsilon, int _evals){
    ESPParameters params;
    params.populationSize = _popsize;
    params.maxGenerations = _maxgen;
    params.nnFormatFilename = _nnFormatFile;
    params.stagnationThreshold = _stag;
    params.fitnessEpsilonThreshold = _epsilon;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = _popsize/10;
    params.sampleEvaluationsPerChromosome = _evals;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    return params;
}

void runSim(GraphicsEngine* _engine, Simulation* _sim, GAType _type, string _inputFile, string _outputFile){
    if(!_sim){
        cout << "Simulation not created" << endl;
        return;
    }

    _sim->initialise();

    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

#ifdef TRAIN
    Solution solution;

    if(rank == 0){
        GeneticAlgorithm* ga;

        if(_type == TYPE_STANDARD){
            StandardGAParameters params = getGAParameters(100, 200, _inputFile, 0, 0);
            ga = new StandardGA(params);
        }
        else{
            ESPParameters params = getESPParameters(40, 200, _inputFile, 9999, 0, 5);
            ga = new ESP(params);
        }

        GAEngine gaengine;
        SimulationContainer cont(_sim);
        solution = gaengine.train(ga, &cont, "");

        cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
        solution.printToFile(_outputFile);

        cont.resetSimulation();
        cont.setSolution(&solution);
        _engine->setSimulation(&cont);
        
        _engine->renderSimulation();

        delete ga;
    }
    else{
        if(_type == TYPE_ESP){
            Slave slave(_sim);

            slave.run();
        }
    }
#else
    if(rank == 0){
        SimulationContainer cont(_sim);
        Solution solution(_outputFile);
        cont.setSolution(&solution);
        
        _engine->setSimulation(&cont);
        
        _engine->renderSimulation();
    }
    else delete _sim;
#endif
}

SimInfo createSimulation(string _simName, GraphicsEngine* _engine){
    int seed = 120;

    if (_simName == "BridgeCarSim") return SimInfo(new BridgeSimulation(2, 10, CAR, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/bridgesimulation/car/input6h.xml", "neuralxmls/bridgesimulation/car/output.xml");
    else if(_simName == "BridgeMouseSim") return SimInfo(new BridgeSimulation(2, 30, MOUSE, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/bridgesimulation/mouse/input6h.xml", "neuralxmls/bridgesimulation/mouse/output.xml");
    else if(_simName == "CorneringSim") return SimInfo(new CorneringSim(2, 4, 450, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/corneringsimulation/input6h.xml", "neuralxmls/corneringsimulation/output.xml");
    else if(_simName == "CarCrashSim") return SimInfo(new CarCrashSimulation(2, 10, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/carcrashsimulation/input6h.xml", "neuralxmls/carcrashsimulation/output.xml");
    else if(_simName == "CarRaceSim") return SimInfo(new CarRaceSimulation(2, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/carracesimulation/input6h_het.xml", "neuralxmls/carracesimulation/output.xml");
    else if(_simName == "WarRobotSim") return SimInfo(new WarRobotSimulation(2, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/warrobotsimulation/input5h.xml", "neuralxmls/warrobotsimulation/output.xml");
    else if(_simName == "MouseEscapeSim") return SimInfo(new MouseEscapeSimulation(2, 450, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/mouseescapesimulation/input6h.xml", "neuralxmls/mouseescapesimulation/output.xml");
    else if(_simName == "MouseScatterSim") return SimInfo(new MouseScatterSimulation(2, 100, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/mousescattersimulation/input6h.xml", "neuralxmls/mousescattersimulation/output.xml");
    else if(_simName == "SFObstacleSim") return SimInfo(new SFObstacleSimulation(2, 40, 450, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfobstaclesimulation/input6h.xml", "neuralxmls/sfobstaclesimulation/output.xml");
    else if(_simName == "SFObstacleFieldSim") return SimInfo(new SFObstaclefieldSimulation(2, 40, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfobstaclefieldsimulation/input6h.xml", "neuralxmls/sfobstaclefieldsimulation/output.xml");
    else if(_simName == "SFTurnbackSim") return SimInfo(new SFTurnbackSimulation(2, 30, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfturnbacksimulation/input6h.xml", "neuralxmls/sfturnbacksimulation/output.xml");
    else if(_simName == "PoleBalanceSim") return SimInfo(new PoleBalancingSimulation(900, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/polebalancing/input.xml", "neuralxmls/polebalancing/output.xml");

    return NULL;
}

int main(int argc, char** argv){
    MPI_Init(&argc, &argv);

    srand(time(0));

    string simName = "MouseScatterSim";
    GraphicsEngine* engine = new GraphicsEngine(NULL);

    SimInfo simInfo = createSimulation(simName, engine);

    runSim(engine, simInfo.get<0>(), TYPE_ESP, simInfo.get<1>(), simInfo.get<2>());
    //testCrossoverOp();

    delete engine;

    MPI_Finalize();

    return 0;
}