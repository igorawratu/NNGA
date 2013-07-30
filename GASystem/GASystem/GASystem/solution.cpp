#include "solution.h"

Solution::Solution(string _filename){
    xmldoc doc;
    pugi::xml_parse_result result = doc.load_file(_filename.c_str());
    if(!result)
        cerr << "Error: unable to parse the file " << _filename << endl;
    else{
        pugi::xml_node root = doc.first_child();
        for(pugi::xml_node currNetwork = root.first_child(); currNetwork; currNetwork = currNetwork.next_sibling()){
            NeuralNetwork currNN;
            if(!currNN.initialize(&currNetwork, true)){
                cerr << "Error: unable to initialize solution properly" << endl;
            }
            else mNeuralNets.push_back(currNN);
        }
    }

}

Solution::Solution(vector<NeuralNetwork> _nets){
    mNeuralNets = _nets;
}
    
vector<vector<double>> Solution::evaluateAllNeuralNetworks(vector<map<uint, double>> _inputs){
    vector<vector<double>> output;
    if(_inputs.size() != mNeuralNets.size()){
        cerr << "Error: inputs do not match neural networks" << endl;
        return output;
    }

    for(uint k = 0; k < mNeuralNets.size(); k++)
        output.push_back(mNeuralNets[k].evaluate(_inputs[k]));

    return output;
}


vector<double> Solution::evaluateNeuralNetwork(uint _index, map<uint, double> _inputs){
    if(_index >= mNeuralNets.size()){
        cerr << "Error: neural network index unknown" << endl;
        return vector<double>();
    }

    return mNeuralNets[_index].evaluate(_inputs);
}

void Solution::printToFile(string _filename){
    xmldoc doc;
    pugi::xml_node root = doc.append_child("NeuralNetworks");

    for(uint k = 0; k < mNeuralNets.size(); k++){
        pugi::xml_node nnNode;
        mNeuralNets[k].getXMLStructure(root);
    }

    doc.save_file(_filename.c_str());
    
}