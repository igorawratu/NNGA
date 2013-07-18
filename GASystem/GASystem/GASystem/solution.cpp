#include "solution.h"

Solution::Solution(string _filename){
    ifstream file;
    file.open(_filename.c_str());
    string line;

    while(getline(file, line)){
        xmldoc doc;
        pugi::xml_parse_result result = doc.load_file(string.c_str());
        if(!result){
            cerr << "Error: unable to parse the file " << line << endl;
            continue;
        }

        mNeuralNets.push_back(NeuralNetwork(&doc, false);
    }

    file.close();
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

}