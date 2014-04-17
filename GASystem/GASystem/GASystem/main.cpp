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

#define TRAIN

using namespace std;

void runBridgeCarSim(){
    int seed = 50;
    GraphicsEngine engine(NULL);

    BridgeSimulation* sim = new BridgeSimulation(2, 10, CAR, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/bridgesimulation/car/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/bridgesimulation/car/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/bridgesimulation/car/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runBridgeMouseSim(){
    int seed = 50;
    GraphicsEngine engine(NULL);

    BridgeSimulation* sim = new BridgeSimulation(2, 30, MOUSE, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/bridgesimulation/mouse/input6h.xml";
    params.stagnationThreshold = 10;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.1;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "UNDX";
    params.selectionAlgorithm = "QuadraticRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/bridgesimulation/mouse/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/bridgesimulation/mouse/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runCorneringSim(){
    int seed = 50;
    GraphicsEngine engine(NULL);

    CorneringSim* sim = new CorneringSim(2, 4, 450, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/corneringsimulation/input6h.xml";
    params.stagnationThreshold = 5;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/corneringsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/corneringsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runCarCrashSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    CarCrashSimulation* sim = new CarCrashSimulation(1, 10, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/carcrashsimulation/input6h.xml";
    params.stagnationThreshold = 10;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.1;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "QuadraticRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/carcrashsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/carcrashsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runCarRaceSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    CarRaceSimulation* sim = new CarRaceSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/carracesimulation/input5h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "QuadraticRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/carracesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/carracesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runWarRobotSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    WarRobotSimulation* sim = new WarRobotSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/warrobotsimulation/input5h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/warrobotsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/warrobotsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}


void runMouseEscapeSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    MouseEscapeSimulation* sim = new MouseEscapeSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/mouseescapesimulation/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/mouseescapesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/mouseescapesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runMouseScatterSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    MouseScatterSimulation* sim = new MouseScatterSimulation(2, 100, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/mousescattersimulation/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/mousescattersimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/mousescattersimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFObstacleSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    SFObstacleSimulation* sim = new SFObstacleSimulation(2, 40, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfobstaclesimulation/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfobstaclesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfobstaclesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFObstacleFieldSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    SFObstaclefieldSimulation* sim = new SFObstaclefieldSimulation(2, 40, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfobstaclefieldsimulation/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfobstaclefieldsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfobstaclefieldsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFTurnbackSim(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    SFTurnbackSimulation* sim = new SFTurnbackSimulation(2, 30, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 50;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfturnbacksimulation/input6h.xml";
    params.stagnationThreshold = 0;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfturnbacksimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfturnbacksimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
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

void runBridgeCarSimESP(){
    int seed = 50;
    GraphicsEngine engine(NULL);

    BridgeSimulation* sim = new BridgeSimulation(2, 10, CAR, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/bridgesimulation/car/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/bridgesimulation/car/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/bridgesimulation/car/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runBridgeMouseSimESP(){
    int seed = 50;
    GraphicsEngine engine(NULL);

    BridgeSimulation* sim = new BridgeSimulation(2, 30, MOUSE, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

#ifdef TRAIN
    Solution solution;

    if(rank == 0){
        SimulationContainer cont(sim);

        ESPParameters params;
        params.populationSize = 20;
        params.maxGenerations = 200;
        params.nnFormatFilename = "neuralxmls/bridgesimulation/mouse/input6h.xml";
        params.stagnationThreshold = 400;
        params.fitnessEpsilonThreshold = 0;
        params.mutationAlgorithm = "GaussianMutation";
        params.mutationParameters["MutationProbability"] = 0.02;
        params.mutationParameters["Deviation"] = 0.2;
        params.mutationParameters["MaxConstraint"] = 1;
        params.mutationParameters["MinConstraint"] = -1;
        params.crossoverAlgorithm = "LX";
        params.selectionAlgorithm = "LRankSelection";
        params.elitismCount = 2;
        params.sampleEvaluationsPerChromosome = 3;
        params.crossoverParameters["CrossoverProbability"] = 0.6;

        GeneticAlgorithm* ga = new ESP(params);

        GAEngine gaengine;
        solution = gaengine.train(ga, &cont, "");

        delete ga;

        cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
        solution.printToFile("neuralxmls/bridgesimulation/mouse/output.xml");

        cont.resetSimulation();

        cont.setSolution(&solution);
        
        engine.setSimulation(&cont);
        
        engine.renderSimulation();
    }
    else{
        Slave slave(sim);

        slave.run();
    }
#else
    if(rank == 0){
        Solution solution("neuralxmls/bridgesimulation/mouse/output.xml");
        cont.setSolution(&solution);
        
        engine.setSimulation(&cont);
        
        engine.renderSimulation();
    }
#endif
}

void runCorneringSimESP(){
    int seed = rand();

    cout << seed << endl;

    GraphicsEngine engine(NULL);

    CorneringSim* sim = new CorneringSim(2, 4, 450, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 40;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/corneringsimulation/input6h.xml";
    params.stagnationThreshold = 400;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "LX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 2;
    params.sampleEvaluationsPerChromosome = 5;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/corneringsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/corneringsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runCarCrashSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    CarCrashSimulation* sim = new CarCrashSimulation(1, 8, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/carcrashsimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.1;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "QuadraticRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/carcrashsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/carcrashsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runCarRaceSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    CarRaceSimulation* sim = new CarRaceSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/carracesimulation/input6h.xml";
    params.stagnationThreshold = 40;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "QuadraticRankSelection";
    params.elitismCount = 2;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.6;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/carracesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/carracesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runWarRobotSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    WarRobotSimulation* sim = new WarRobotSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/warrobotsimulation/input5h.xml";
    params.stagnationThreshold = 40;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 2;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/warrobotsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/warrobotsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}


void runMouseEscapeSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    MouseEscapeSimulation* sim = new MouseEscapeSimulation(2, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/mouseescapesimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/mouseescapesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/mouseescapesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runMouseScatterSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    MouseScatterSimulation* sim = new MouseScatterSimulation(2, 100, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/mousescattersimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/mousescattersimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/mousescattersimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFObstacleSimESP(){
    int seed = rand();
    GraphicsEngine engine(NULL);

    SFObstacleSimulation* sim = new SFObstacleSimulation(2, 40, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfobstaclesimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfobstaclesimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfobstaclesimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFObstacleFieldSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    SFObstaclefieldSimulation* sim = new SFObstaclefieldSimulation(2, 40, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfobstaclefieldsimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfobstaclefieldsimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfobstaclefieldsimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runSFTurnbackSimESP(){
    int seed = 110;
    GraphicsEngine engine(NULL);

    SFTurnbackSimulation* sim = new SFTurnbackSimulation(2, 30, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 20;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/sfturnbacksimulation/input6h.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "LRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/sfturnbacksimulation/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/sfturnbacksimulation/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

void runPBESP(){
    int seed = 10;
    GraphicsEngine engine(NULL);

    PoleBalancingSimulation* sim = new PoleBalancingSimulation(900, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    SimulationContainer cont(sim);

#ifdef TRAIN
    ESPParameters params;
    params.populationSize = 40;
    params.maxGenerations = 1000;
    params.nnFormatFilename = "neuralxmls/polebalancing/input.xml";
    params.stagnationThreshold = 20;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.02;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "BLX";
    params.selectionAlgorithm = "NLRankSelection";
    params.elitismCount = 5;
    params.sampleEvaluationsPerChromosome = 3;
    params.crossoverParameters["CrossoverProbability"] = 0.8;

    GeneticAlgorithm* ga = new ESP(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont, "");

    delete ga;

    cout << "FINAL TRAINED FITNESS: " << solution.fitness() << endl;
    solution.printToFile("neuralxmls/polebalancing/output.xml");

    cont.resetSimulation();
#else
    Solution solution("neuralxmls/polebalancing/output.xml");
#endif

    cont.setSolution(&solution);
    
    engine.setSimulation(&cont);
    
    engine.renderSimulation();
}

int main(int argc, char** argv){
    MPI_Init(&argc, &argv);

    srand(time(0));

    //testANNSerialization();
    //testSolutionSerialization();

    //runBridgeMouseSim();
    //runBridgeCarSim();
    //runCarCrashSim();
    //runCarRaceSim();
    //runWarRobotSim();
    //runCorneringSim();
    //runMouseEscapeSim();
    //runMouseScatterSim();
    //runSFObstacleSim();
    //runSFTurnbackSim();
    //runSFObstacleFieldSim();
    runBridgeMouseSimESP();
    //runBridgeCarSimESP();
    //runCarCrashSimESP();
    //runCarRaceSimESP();
    //runWarRobotSimESP();
    //runCorneringSimESP();
    //runMouseEscapeSimESP();
    //runMouseScatterSimESP();
    //runSFObstacleSimESP();
    //runSFTurnbackSimESP();
    //runSFObstacleFieldSimESP();
    //runPBESP();

    //testCrossoverOp();

    MPI_Finalize();

    return 0;
}