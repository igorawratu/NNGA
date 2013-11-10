#include "mutation.h"

void Mutation::conformWeights(vector<double>& _weights, double _maxConstraint, double _minConstraint){
    for(uint k = 0; k < _weights.size(); k++){
        if(_weights[k] > _maxConstraint)
            _weights[k] = _maxConstraint;
        if(_weights[k] < _minConstraint)
            _weights[k] = _minConstraint;
    }
}