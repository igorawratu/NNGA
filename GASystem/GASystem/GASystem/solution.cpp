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

    for(uint k = 0; k < mNeuralNets.size(); k++)
        mNeuralNets[k].getXMLStructure(root);

    doc.save_file(_filename.c_str());
    
}

Solution::Solution(int* _nodes, int* _format, float* _weights, int _formatSize, int _weightSize){
    int currNodesPos = 0;
    int ANNCount = _nodes[currNodesPos++];

    int currWeightPos = 0, currFormatPos = 0;

    for(uint k = 0; k < ANNCount; ++k){
        int* currNNNodes = new int[3];
        currNNNodes[0] = _nodes[currNodesPos++];
        currNNNodes[1] = _nodes[currNodesPos++];
        currNNNodes[2] = _nodes[currNodesPos++];

        int lastWeightPos = currWeightPos;
        int lastFormatPos = currFormatPos;
        
        for(uint i = 0; i < currNNNodes[1] + currNNNodes[2]; ++i){
            currFormatPos += _format[currFormatPos + 2] + 3;
            currWeightPos += _format[currFormatPos + 2] + 1;
        }

        int weightSize = currWeightPos - lastWeightPos;
        int formatSize = currFormatPos - lastFormatPos;

        int* currNNFormat = new int[formatSize];
        float* currNNWeight = new float[weightSize];

        for(uint i = 0; i < formatSize; ++i)
            currNNFormat[i] = _format[lastFormatPos + i];

        for(uint i = 0; i < weightSize; ++i)
            currNNWeight[i] = _format[lastWeightPos + i];

        mNeuralNets.push_back(NeuralNetwork(currNNNodes, currNNFormat, currNNWeight, formatSize, weightSize));
    }

    delete [] _nodes;
    delete [] _format;
    delete [] _weights;
}

void Solution::serialize(int*& _nodes, int*& _format, float*& _weights, int& _formatSize, int& _weightSize){
    int nodeSize = mNeuralNets.size() * 3 + 1;
    _nodes = new int[nodeSize];
    int nodePos = 0;
    _formatSize = _weightSize = 0;

    _nodes[nodePos++] = mNeuralNets.size();

    vector<int*> formats;
    vector<float*> weights;
    vector<int> nnFormatSizes;
    vector<float> nnWeightSizes;

    for(uint k = 1; k < mNeuralNets.size(); ++k){
        int *nnNodes, *nnFormats;
        float* nnWeights;
        int nnWS, nnFS;

        mNeuralNets[k - 1].serialize(nnNodes, nnFormats, nnWeights, nnFS, nnWS);

        formats.push_back(nnFormats);
        weights.push_back(nnWeights);

        _nodes[nodePos++] = nnNodes[0];
        _nodes[nodePos++] = nnNodes[1];
        _nodes[nodePos++] = nnNodes[2];

        _formatSize += nnFS;
        _weightSize += nnWS;

        nnFormatSizes.push_back(nnFS);
        nnWeightSizes.push_back(nnWS);
    }

    _format = new int[_formatSize];
    _weights = new float[_weightSize];

    int formatPos = 0, weightPos = 0;

    for(uint k = 0; k < nnFormatSizes.size(); ++k){
        for(uint i = 0; i < nnFormatSizes[k]; ++i)
            _format[formatPos++] = formats[k][i];
        
        for(uint i = 0; i < nnWeightSizes[k]; ++i)
            _weights[weightPos++] = weights[k][i];

        delete [] formats[k];
        delete [] weights[k];
    }
}