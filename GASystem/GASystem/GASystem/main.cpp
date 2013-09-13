#include <iostream>
#include <map>
#include <string>
#include <typeinfo>
#include <vector>

#include "common.h"
#include "neuralnetwork.h"
#include "pugixml.hpp"
#include "nnchromosome.h"
#include "solution.h"
#include "gaussianmutation.h"
#include "common.h"
#include "multipointcrossover.h"
#include "rankselection.h"
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

#define TRAIN

using namespace std;

void quicksort(vector<Chromosome*>& elements, int left, int right)
{
	int i = left;
	int j = right;

	Chromosome* pivot = elements[(left+ right) / 2];
	do
	{
		while (elements[i]->fitness() < pivot->fitness())
			i++;
		while (elements[j]->fitness() > pivot->fitness())
			j--;

		if (i <= j)
		{
			Chromosome* temp = elements[i]; elements[i] = elements[j]; elements[j] = temp;
			i++; j--;
		}
	} while (i <= j);

	if (left < j)
		quicksort(elements, left, j);
	if (i < right)
		quicksort(elements, i, right);
}

void testLoadStore(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\inputoutput\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    xmldoc out;
    pugi::xml_node outRoot = out.append_child("NeuralNetworks");
    test.getXMLStructure(outRoot);

    out.save_file("neuralxmls\\inputoutput\\output.xml");
}

void testLoadStoreFixed(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\inputoutputfixed\\iofixed.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    xmldoc out;
    pugi::xml_node outRoot = out.append_child("NeuralNetworks");
    test.getXMLStructure(outRoot);

    out.save_file("neuralxmls\\inputoutputfixed\\iofixedoutput.xml");
}

void testSigmoidOutput(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\sigmoid\\sigmoid.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    map<uint, double> inputs;
    inputs[1] = 0;
    inputs[2] = 0;
    inputs[3] = 0;
    inputs[4] = 0;

    vector<double> nnOutput = test.evaluate(inputs);
    assert(nnOutput.size() == 4);
    for(uint k = 0; k < nnOutput.size(); k++)
        assert(nnOutput[k] == 0.5);
}

void testHidden(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\hidden\\hidden.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    xmldoc out;
    pugi::xml_node outRoot = out.append_child("NeuralNetworks");
    test.getXMLStructure(outRoot);

    out.save_file("neuralxmls\\hidden\\hiddenoutput.xml");
}

void testLoop(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\loop\\loop.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    assert(!test.initialize(&nnRoot, true));
}

void testCopyAssignment(){
    xmldoc outCopy, outAss;
    NeuralNetwork assignment;
    NeuralNetwork* copy;

    if(true){
        xmldoc doc;
        pugi::xml_parse_result result = doc.load_file("neuralxmls\\copyassignment\\original.xml");
        pugi::xml_node root = doc.child("NeuralNetworks");
        pugi::xml_node nnRoot = root.first_child();
        NeuralNetwork test;
        test.initialize(&nnRoot, true);

        copy = new NeuralNetwork(test);
        assignment = test;
    }
    
    pugi::xml_node copyRoot = outCopy.append_child("NeuralNetworks");
    pugi::xml_node assRoot = outAss.append_child("NeuralNetworks");
    copy->getXMLStructure(copyRoot);
    assignment.getXMLStructure(assRoot);

    outAss.save_file("neuralxmls\\copyassignment\\assignment.xml");
    outCopy.save_file("neuralxmls\\copyassignment\\copy.xml");

    delete copy;
}

void testWeightsAndStructure(){
    xmldoc doc, mod, outW, outS;
    NeuralNetwork structNN, weightNN, test;

    pugi::xml_parse_result result = doc.load_file("neuralxmls\\weightsstructure\\original.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();

    test.initialize(&nnRoot, true);

    result = mod.load_file("neuralxmls\\weightsstructure\\modified.xml");
    pugi::xml_node rootS = mod.child("NeuralNetworks");
    pugi::xml_node nnRootS = rootS.first_child();

    weightNN.initialize(&nnRootS, true);
    
    structNN.setStructure(test.getMapStructure());
    weightNN.setWeights(test.getWeights());

    pugi::xml_node strRoot = outS.append_child("NeuralNetworks");
    pugi::xml_node weightRoot = outW.append_child("NeuralNetworks");
    
    weightNN.getXMLStructure(weightRoot);
    structNN.getXMLStructure(strRoot);

    outW.save_file("neuralxmls\\weightsstructure\\weight.xml");
    outS.save_file("neuralxmls\\weightsstructure\\structure.xml");
}

void testHiddenSigmoidOutput(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\hiddensigmoid\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    map<uint, double> inputs;
    inputs[1] = 0;
    inputs[2] = 0;
    inputs[3] = 0;
    inputs[4] = 0;

    vector<double> nnOutput = test.evaluate(inputs);
    assert(nnOutput.size() == 4);
    for(uint k = 0; k < nnOutput.size(); k++)
        assert(nnOutput[k] == 0.5);
}

void testSolutionLoadWrite(){
    Solution solution("neuralxmls\\solution\\input.xml");
    solution.printToFile("neuralxmls\\solution\\output.xml");
}

void testSolutionEvaluation(){
    Solution solution("neuralxmls\\solution\\input.xml");

    map<uint, double> inputs;
    inputs[1] = 0;
    inputs[2] = 0;
    inputs[3] = 0;
    inputs[4] = 0;

    vector<map<uint, double>> inputmap;
    inputmap.push_back(inputs);
    inputmap.push_back(inputs);

    vector<vector<double>> eval = solution.evaluateAllNeuralNetworks(inputmap);
    assert(eval.size() == 2);
    for(uint k = 0; k < eval.size(); k++){
        assert(eval[k].size() == 4);
        for(uint i = 0; i < eval[k].size(); i++)
            assert(eval[k][i] == 0.5);
    }
}

void testChromosomeLoad(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\nnchromosome\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");

    NNChromosome cr;
    assert(cr.initialize(&root));

    Solution sol(cr.getNeuralNets());
    sol.printToFile("neuralxmls\\nnchromosome\\output.xml");
}

void testChromosomeCopyAss(){
    NNChromosome ass;
    NNChromosome* copy;

    if(true){
        xmldoc doc;
        pugi::xml_parse_result result = doc.load_file("neuralxmls\\nnchromosome\\get.xml");
        pugi::xml_node root = doc.child("NeuralNetworks");

        NNChromosome cr;
        assert(cr.initialize(&root));

        ass = cr;
        copy = new NNChromosome(cr);
    }

    Solution assSol(ass.getNeuralNets());
    Solution copySol(copy->getNeuralNets());
    delete copy;
    assSol.printToFile("neuralxmls\\nnchromosome\\assignment.xml");
    copySol.printToFile("neuralxmls\\nnchromosome\\copy.xml");
}

void testChromosomeGetSet(){
    xmldoc doc, mod;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\nnchromosome\\input.xml");
    result = mod.load_file("neuralxmls\\nnchromosome\\modified.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node modroot = mod.child("NeuralNetworks");

    NNChromosome cr;
    cr.initialize(&root);

    NNChromosome structure;
    structure.setStructure(cr.getFullStructureData());

    NNChromosome weight;
    weight.initialize(&modroot);

    weight.setWeights(cr.getWeightData());

    Solution strSol(structure.getNeuralNets()), weightSol(weight.getNeuralNets());

    strSol.printToFile("neuralxmls\\nnchromosome\\structure.xml");
    weightSol.printToFile("neuralxmls\\nnchromosome\\weight.xml");
}

void testGaussianMutation(){
    GaussianMutation mut;
    map<string, double> params;
    params["MutationProbability"] = 0.1;
    params["Deviation"] = 0.2;
    params["MaxConstraint"] = 1;
    params["MinConstraint"] = -1;

    vector<double> weights;
    for(uint k = 0; k < 100; k++)
        weights.push_back(0);

    mut.execute(weights, params);

    cout << "MUTATION TEST OUTPUT: " << endl;
    for(uint k = 0; k < weights.size(); k++)
        cout << weights[k] << " ";
    cout << endl;
}

void testRankSelection(){
    cout << "RANK SELECTION TEST OUTPUT: " << endl;
    vector<Chromosome*> selectionPool;
    for(uint k = 0; k < 100; k++){
        Chromosome* chr = new NNChromosome();
        chr->fitness() = k;
        selectionPool.push_back(chr);
    }
    vector<Chromosome*> selected;
    RankSelection rankselection;
    selected = rankselection.execute(selectionPool, 50, selectionPool);

    assert(selectionPool.size() == 50);
    assert(selected.size() == 50);

    quicksort(selectionPool, 0, selectionPool.size() - 1);
    quicksort(selected, 0, selected.size() - 1);
    
    cout << "Selected: " << endl;
    for(uint k = 0; k < selected.size(); k++){
        cout << selected[k]->fitness() << " ";
        delete selected[k];
    }
    selected.clear();
    cout << endl;

    cout << "Remaining: " << endl;
    for(uint k = 0; k < selectionPool.size(); k++){
        cout << selectionPool[k]->fitness() << " ";
        delete selectionPool[k];
    }
    selectionPool.clear();
    cout << endl;
}

void testFactories(){
    GaussianMutation gMut;
    RankSelection rSel;
    MultipointCrossover mpCross;

    Mutation* mut = MutationFactory::instance().create("GaussianMutation");
    Selection* sel = SelectionFactory::instance().create("RankSelection");
    Crossover* cross = CrossoverFactory::instance().create("MultipointCrossover");

    assert(typeid(gMut) == typeid(*mut));
    assert(typeid(rSel) == typeid(*sel));
    assert(typeid(mpCross) == typeid(*cross));

    delete mut;
    delete sel;
    delete cross;

    Mutation* mutNul = MutationFactory::instance().create("somerandommut");
    Selection* selNul = SelectionFactory::instance().create("somerandomsel");
    Crossover* crossNul = CrossoverFactory::instance().create("somerandomco");

    assert(mutNul == 0);
    assert(crossNul == 0);
    assert(selNul == 0);
}

void testMultipointCrossover(){
    MultipointCrossover mpco;
    
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\nnchromosome\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");

    vector<Chromosome*> population;
    population.push_back(new NNChromosome());
    population.push_back(new NNChromosome());
    dynamic_cast<NNChromosome*>(population[0])->initialize(&root);
    dynamic_cast<NNChromosome*>(population[1])->initialize(&root);
    
    vector<Chromosome*> offspring = mpco.execute(population, 1, map<string, double>());
    Solution p1(dynamic_cast<NNChromosome*>(population[0])->getNeuralNets()), p2(dynamic_cast<NNChromosome*>(population[1])->getNeuralNets()), off(dynamic_cast<NNChromosome*>(offspring[0])->getNeuralNets());
    p1.printToFile("neuralxmls\\nnchromosome\\p1.xml");
    p2.printToFile("neuralxmls\\nnchromosome\\p2.xml");
    off.printToFile("neuralxmls\\nnchromosome\\off.xml");
}

void testGA(){
    StandardGAParameters params;

    params.populationSize = 50;
    params.maxGenerations = 100;
    params.nnFormatFilename = "neuralxmls\\nnchromosome\\input.xml";
    params.stagnationThreshold = 1;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.1;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "MultipointCrossover";
    params.selectionAlgorithm = "RankSelection";

    GeneticAlgorithm* ga = new StandardGA(params);
    Simulation* sim = new DummySimulation(100, 5, 5, NULL);
    vector<Fitness*> fit;
    fit.push_back(new DummyFitness());

    //create sim container ere
    SimulationContainer sc(sim, fit);

    GAEngine gaengine;
    Solution sol = gaengine.train(ga, &sc);

    delete sim;
    delete fit[0];
    fit.clear();
    delete ga;
    sol.printToFile("neuralxmls\\integration\\solution.xml");
}


void runNNTests(){
    srand(time(0));
    testLoadStore();
    testLoadStoreFixed();
    testSigmoidOutput();
    testHidden();
    testLoop();
    testCopyAssignment();
    testWeightsAndStructure();
    testHiddenSigmoidOutput();
}

void runGATests(){
    srand(time(0));
    testSolutionLoadWrite();
    testSolutionEvaluation();
    testChromosomeLoad();
    testChromosomeCopyAss();
    testChromosomeGetSet();
    testGaussianMutation();
    testRankSelection();
    testFactories();
    testMultipointCrossover();
}

int main(){
    //srand(time(0));
    GraphicsEngine engine(NULL);

    /*vector<vector3> waypoints;
    waypoints.push_back(vector3(-44, -4, 50));
    waypoints.push_back(vector3(-48, -4, -48));
    waypoints.push_back(vector3(62, -4, -50));*/
    Line finishLine;
    finishLine.p1 = vector3(-7, -47, -25);
    finishLine.p2 = vector3(7, -47, -25);
    
    int seed = rand();
    cout << seed << endl;

    BridgeSimulation* sim = new BridgeSimulation(5, 10, finishLine, CAR, 300, 5, 30, NULL, engine.getResourceManager(), seed);
    sim->initialise();

    vector<Fitness*> fitList;
    //fitList.push_back(new WaypointFitness(waypoints));
    fitList.push_back(new CollisionFitness());
    fitList.push_back(new FinishLineFitness(false, finishLine));

    SimulationContainer cont(sim, fitList);

#ifdef TRAIN
    StandardGAParameters params;
    params.populationSize = 100;
    params.maxGenerations = 200;
    params.nnFormatFilename = "neuralxmls/bridgesimulation/car/input4h.xml";
    params.stagnationThreshold = 10;
    params.fitnessEpsilonThreshold = 0;
    params.mutationAlgorithm = "GaussianMutation";
    params.mutationParameters["MutationProbability"] = 0.1;
    params.mutationParameters["Deviation"] = 0.2;
    params.mutationParameters["MaxConstraint"] = 1;
    params.mutationParameters["MinConstraint"] = -1;
    params.crossoverAlgorithm = "MultipointCrossover";
    params.selectionAlgorithm = "RankSelection";
    params.elitismCount = 5;

    GeneticAlgorithm* ga = new StandardGA(params);

    GAEngine gaengine;
    Solution solution = gaengine.train(ga, &cont);

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

    for(int k = 0; k < fitList.size(); k++)
        delete fitList[k];
    fitList.clear();

    return 0;
}