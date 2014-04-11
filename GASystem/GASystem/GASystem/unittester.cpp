#include "unittester.h"

/*void UnitTester::quicksort(vector<Chromosome*>& elements, int left, int right)
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

    xmldoc doc2;
    pugi::xml_parse_result result2 = doc2.load_file("neuralxmls\\inputoutput\\output.xml");
    pugi::xml_node root2 = doc2.child("NeuralNetworks");
    pugi::xml_node nnRoot2 = root2.first_child();
    NeuralNetwork in;
    in.initialize(&nnRoot2, true);

    CPPUNIT_ASSERT(test == in);
}

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

    CPPUNIT_ASSERT(test == outANN);
}

void testSolutionSerialization(){
    Solution solution("neuralxmls\\solution\\input.xml");
    solution.printToFile("neuralxmls\\solution\\output.xml");

    int weightSize, formatSize;
    int *format, *nodes;
    double *weights;

    solution.serialize(nodes, format, weights, formatSize, weightSize);

    Solution serSol(nodes, format, weights, formatSize, weightSize);

    CPPUNIT_ASSERT(solution == serSol);
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
        CPPUNIT_ASSERT(nnOutput[k] == 0.5);
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
    CPPUNIT_ASSERT(!test.initialize(&nnRoot, true));
}

void testCopyAssignment(){
    xmldoc outCopy, outAss;
    NeuralNetwork assignment;
    NeuralNetwork* copy;

    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\copyassignment\\original.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");
    pugi::xml_node nnRoot = root.first_child();
    NeuralNetwork test;
    test.initialize(&nnRoot, true);

    copy = new NeuralNetwork(test);
    assignment = test;

    CPPUNIT_ASSERT(*copy == test);
    CPPUNIT_ASSERT(assignment == test);

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

    CPPUNIT_ASSERT(test == structNN);
    CPPUNIT_ASSERT(test == weightNN);
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
        CPPUNIT_ASSERT(nnOutput[k] == 0.5);
}

void testSolutionLoadWrite(){
    Solution solution("neuralxmls\\solution\\input.xml");
    solution.printToFile("neuralxmls\\solution\\output.xml");

    Solution solution2("neuralxmls\\solution\\output.xml");

    CPPUNIT_ASSERT(solution == solution2);
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
            CPPUNIT_ASSERT(eval[k][i] == 0.5);
    }
}

void testChromosomeLoad(){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file("neuralxmls\\nnchromosome\\input.xml");
    pugi::xml_node root = doc.child("NeuralNetworks");

    NNChromosome cr;
    CPPUNIT_ASSERT(cr.initialize(&root));

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
    
    CPPUNIT_ASSERT(assSol == copySol);

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

void testFactories(){
    GaussianMutation gMut;
    QuadraticRankSelection rSel;
    MultipointCrossover mpCross;

    Mutation* mut = MutationFactory::instance().create("GaussianMutation");
    Selection* sel = SelectionFactory::instance().create("RankSelection");
    Crossover* cross = CrossoverFactory::instance().create("MultipointCrossover");

    CPPUNIT_ASSERT(typeid(gMut) == typeid(*mut));
    CPPUNIT_ASSERT(typeid(rSel) == typeid(*sel));
    CPPUNIT_ASSERT(typeid(mpCross) == typeid(*cross));

    delete mut;
    delete sel;
    delete cross;

    Mutation* mutNul = MutationFactory::instance().create("somerandommut");
    Selection* selNul = SelectionFactory::instance().create("somerandomsel");
    Crossover* crossNul = CrossoverFactory::instance().create("somerandomco");

    CPPUNIT_ASSERT(mutNul == 0);
    CPPUNIT_ASSERT(crossNul == 0);
    CPPUNIT_ASSERT(selNul == 0);
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
    
    vector<Chromosome*> offspring = mpco.execute(population, 1, map<string, double>(), SelectionFactory::instance().create("RankSelection"));
    Solution p1(dynamic_cast<NNChromosome*>(population[0])->getNeuralNets()), p2(dynamic_cast<NNChromosome*>(population[1])->getNeuralNets()), off(dynamic_cast<NNChromosome*>(offspring[0])->getNeuralNets());
    p1.printToFile("neuralxmls\\nnchromosome\\p1.xml");
    p2.printToFile("neuralxmls\\nnchromosome\\p2.xml");
    off.printToFile("neuralxmls\\nnchromosome\\off.xml");
}*/