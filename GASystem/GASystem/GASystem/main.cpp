#include <iostream>
#include <map>

#include "neuralnetwork.h"
#include "pugixml.hpp"

using namespace std;

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
    for(int k = 0; k < nnOutput.size(); k++)
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
    for(int k = 0; k < nnOutput.size(); k++)
        assert(nnOutput[k] == 0.5);
}

void runTests(){
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

int main(){
    runTests();

    int x;
    cin >> x;

    return 0;
}