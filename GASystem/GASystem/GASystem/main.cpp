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
#include "evacuationsimulation.h"
#include "esp.h"
#include "polebalancingsimulation.h"
#include <boost/lexical_cast.hpp>
#include "slave.h"
#include <boost/tuple/tuple.hpp>
#include "cmaes.h"
#include "cmaesparameters.h"

//#define TRAIN

using namespace std;

enum GAType{TYPE_STANDARD, TYPE_ESP, TYPE_CMAES};

typedef boost::tuples::tuple<Simulation*, string, string> SimInfo;


void testANNSerialization(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\inputoutput\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    int weightSize, formatSize;
    int *nodes, *format;
    double* weights;
    test.serialize(nodes, format, weights, formatSize, weightSize);

    NeuralNetwork outANN(nodes, format, weights, formatSize, weightSize);

    xmldoc outser;
    pugi::xml_node outRootSer = outser.append_child("NeuralNetworks");
    outANN.getXMLStructure(outRootSer);

    xmldoc out;
    pugi::xml_node outRoot = out.append_child("NeuralNetworks");
    test.getXMLStructure(outRoot);

	out.save_file("neuralxmls\\inputoutput\\out_ser.xml");
}


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
            StandardGAParameters params = _sim->getSGAParameters(_inputFile);
            ga = new StandardGA(params);
        }
        else if(_type == TYPE_ESP){
            ESPParameters params = _sim->getESPParams(_inputFile);
            ga = new ESP(params);
        }
        else if(_type == TYPE_CMAES){
            CMAESParameters params = _sim->getCMAESParameters(_inputFile);
            ga = new CMAES(params);
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
        Slave slave(_sim);

        slave.run();
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
    else if(_simName == "CorneringSim") return SimInfo(new CorneringSim(2, 4, 400, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/corneringsimulation/input6h.xml", "neuralxmls/corneringsimulation/output.xml");
    else if(_simName == "CarCrashSim") return SimInfo(new CarCrashSimulation(2, 10, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/carcrashsimulation/input6h.xml", "neuralxmls/carcrashsimulation/output.xml");
    else if(_simName == "CarRaceSim") return SimInfo(new CarRaceSimulation(2, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/carracesimulation/input6h_het.xml", "neuralxmls/carracesimulation/output.xml");//change env
    else if(_simName == "WarRobotSim") return SimInfo(new WarRobotSimulation(2, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/warrobotsimulation/input6h.xml", "neuralxmls/warrobotsimulation/output.xml");
    else if(_simName == "MouseEscapeSim") return SimInfo(new MouseEscapeSimulation(2, 450, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/mouseescapesimulation/input6h.xml", "neuralxmls/mouseescapesimulation/output.xml");
    else if(_simName == "SFObstacleSim") return SimInfo(new SFObstacleSimulation(2, 40, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfobstaclesimulation/input6h.xml", "neuralxmls/sfobstaclesimulation/output.xml");
    else if(_simName == "SFObstacleFieldSim") return SimInfo(new SFObstaclefieldSimulation(2, 40, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfobstaclefieldsimulation/input6h.xml", "neuralxmls/sfobstaclefieldsimulation/output.xml");
    else if(_simName == "SFTurnbackSim") return SimInfo(new SFTurnbackSimulation(2, 40, 300, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/sfturnbacksimulation/input6h.xml", "neuralxmls/sfturnbacksimulation/output.xml");
    else if(_simName == "PoleBalanceSim") return SimInfo(new PoleBalancingSimulation(900, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/polebalancing/input.xml", "neuralxmls/polebalancing/output.xml");
    else if(_simName == "Dummy") return SimInfo(new DummySimulation(300, 5, 30, _engine->getResourceManager()), "", "");
    else if(_simName == "EvacuationSimulation") return SimInfo(new EvacuationSimulation(2, 60, 350, 5, 30, NULL, _engine->getResourceManager(), seed), "neuralxmls/evacuationsimulation/input6h.xml", "neuralxmls/evacuationsimulation/output.xml");

    return NULL;
}

int main(int argc, char** argv){
    MPI_Init(&argc, &argv);

    srand(time(0));

    string simName = "CarRaceSim";

    GraphicsEngine* engine = new GraphicsEngine(NULL);

    SimInfo simInfo = createSimulation(simName, engine);

    runSim(engine, simInfo.get<0>(), TYPE_CMAES, simInfo.get<1>(), simInfo.get<2>());
    //testCrossoverOp();

    delete engine;

    MPI_Finalize();

    return 0;
}