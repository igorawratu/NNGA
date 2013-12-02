#include "pcx.h"

PCX::PCX(){}
PCX::~PCX(){}

void PCX::calculateDGParents(const vector<Chromosome*>& _parents, vector<double>& _d, vector<double>& _g, vector<double>& _p1, vector<double>& _p2, vector<double>& _p3){
    vector<map<uint, vector<double>>> p1Weights, p2Weights, p3Weights;
    p1Weights = _parents[0]->getWeightData(); p2Weights = _parents[1]->getWeightData(); p3Weights = _parents[2]->getWeightData();
        
    //calculate line d(distance vector from p1 to p2) and m(midpoint of p1 and p2), also put other stuff in normal vector to make things easier to code
    for(uint k = 0; k < p1Weights.size(); ++k){
        for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
            for(uint i = 0; i < iter->second.size(); i++){
                _g.push_back((p1Weights[k][iter->first][i] + p2Weights[k][iter->first][i] + p3Weights[k][iter->first][i]) / 3);
                _d.push_back(p1Weights[k][iter->first][i] - _g[k]);
                _p1.push_back(p1Weights[k][iter->first][i]);
                _p2.push_back(p2Weights[k][iter->first][i]);
                _p3.push_back(p3Weights[k][iter->first][i]);
            }
        }
    }
}

double PCX::calculateMeanDistanceD(const vector<double>& _dvec, const vector<double>& _g, const vector<double>& _p2vec, const vector<double>& _p3vec){
    //calculate projection of p3 onto line d, and then calc the distance between the projection and p3
    double dottopp3 = 0, dottopp2 = 0, dotbot = 0, distance = 0;
        
    for(uint k = 0; k < _dvec.size(); ++k){
        dotbot += _dvec[k] * _dvec[k];
        dottopp3 += (_p3vec[k] - _g[k]) * _dvec[k];
        dottopp2 += (_p2vec[k] - _g[k]) * _dvec[k];
    }
    
    double p1distance = 0, p2distance = 0;
    for(uint k = 0; k < _dvec.size(); ++k){
        double temp = (_dvec[k] * dottopp3/dotbot) - (_p3vec[k] - _g[k]);
        p1distance += temp * temp;
    }
    distance += sqrt(p1distance);

    for(uint k = 0; k < _dvec.size(); ++k){
        double temp = (_dvec[k] * dottopp2/dotbot) - (_p2vec[k] - _g[k]);
        p2distance += temp * temp;
    }
    distance += sqrt(p2distance);
    distance /= 2;
    
    return distance;
}

void PCX::calculateOrthogonalBasis(double _spanvecdim, double _spansize, double** _initialspan, double** _orthbasisvectors, const vector<double>& _dvec){
    //initialise span and orthobasis matrices
    for(uint k = 0; k < _spansize; ++k){
        for(uint i = 0; i < _spanvecdim; ++i){
            _initialspan[k][i] = 0;
            _orthbasisvectors[k][i] = 0;
        }
    }

    //calculate the span of the subspace perpendicular to d
    for(uint k = 0; k < _spanvecdim; ++k){
        if(!k)
            continue;

        _initialspan[k - 1][0] = _dvec[k] / (-_dvec[0] == 0 ? 0.0001 : -_dvec[0]);
        _initialspan[k - 1][k] = 1;
    }

    //run Gram-Schmidt Process to find orthonormal basis of subspace defined by span
    for(uint k = 0; k < _spansize; ++k){
        for(uint i = 0; i < _spanvecdim; ++i){
            _orthbasisvectors[k][i] = _initialspan[k][i];
        }

        for(uint i = 0; i < k; ++i){
            double pointDot = 0, lineDot = 0;
            for(uint j = 0; j < _spanvecdim; ++j){
                pointDot += _orthbasisvectors[i][j] * _orthbasisvectors[k][j];
                lineDot += _orthbasisvectors[i][j] * _orthbasisvectors[i][j];
            }

            for(uint j = 0; j < _spanvecdim; ++j){
                _orthbasisvectors[k][j] -= _orthbasisvectors[i][j] * pointDot/lineDot;
            }
        }
    }

    //normalise the orthogonal vectors
    for(uint k = 0; k < _spansize; ++k){
        double distance = 0;
        for(uint i = 0; i < _spanvecdim; ++i)
            distance += _orthbasisvectors[k][i] * _orthbasisvectors[k][i];

        distance = sqrt(distance);

        for(uint i = 0; i < _spanvecdim; ++i)
            _orthbasisvectors[k][i] /= distance;
    }
}


vector<Chromosome*> PCX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters, Selection* _selectionAlgorithm){
    vector<Chromosome*> offspring;

    boost::mt19937 rngCP(rand());
    boost::uniform_real<double> distCP(0, 1);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genCP(rngCP, distCP);

    while(offspring.size() < numOffspring){
        vector<map<uint, vector<double>>> p1Weights, childWeights;
        vector<double> gvec, dvec, p1vec, p2vec, p3vec;

        vector<Chromosome*> parents = _selectionAlgorithm->execute(_population, 3, vector<Chromosome*>());

        if(genCP() > _parameters["CrossoverProbability"]){
            boost::mt19937 rngP(rand());
            boost::uniform_int<> distP(0, 2);
            boost::variate_generator<boost::mt19937, boost::uniform_int<>> genParent(rngP, distP);

            offspring.push_back(parents[genParent()]->clone());
        }
        else{
            p1Weights = parents[0]->getWeightData();

            calculateDGParents(parents, dvec, gvec, p1vec, p2vec, p3vec);

            double distance = calculateMeanDistanceD(dvec, gvec, p2vec, p3vec);

            
            const int spansize = dvec.size() - 1;
            const int spanvecdim = dvec.size();
            double **initialspan = new double*[spansize];
            double **orthbasisvectors = new double*[spansize];      
            double *sum = new double[spanvecdim];

            for(uint k = 0; k < spansize; ++k){
                initialspan[k] = new double[spanvecdim];
                orthbasisvectors[k] = new double[spanvecdim];
            }

            calculateOrthogonalBasis(spanvecdim, spansize, initialspan, orthbasisvectors, dvec);

            //rngs
            double sigone = (0.35 * 0.35)/(double)dvec.size();
            double sigtwo = 0.25;

            boost::mt19937 rng(rand());
            boost::normal_distribution<> dist(0, sigone);
            boost::variate_generator<boost::mt19937, boost::normal_distribution<double>> n1(rng, dist);

            boost::mt19937 rng2(rand());
            boost::normal_distribution<> dist2(0, sigtwo);
            boost::variate_generator<boost::mt19937, boost::normal_distribution<double>> n2(rng2, dist2);

            //calculate sum of the orthogonal basis weighted with random values sampled from n2(sampled per vector)
            for(uint k = 0; k < spanvecdim; ++k)
                sum[k] = 0;

            for(uint k = 0; k < spanvecdim; ++k){
                double rval = n1();

                for(uint i = 0; i < spansize; ++i)
                    sum[k] += orthbasisvectors[i][k] * rval;
            }


            //run the actual undx equation
            for(uint k = 0; k < p1Weights.size(); k++){
                map<uint, vector<double>> currentNetworkWeights;
                uint mapPos = 0;
                for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                    vector<double> weights;
                    for(uint i = 0; i < iter->second.size(); i++){
                        uint currWeightPos = k+mapPos+i;
                        double weightVal = p1vec[currWeightPos] + dvec[currWeightPos] * n2() + distance * sum[currWeightPos];
                        //cout << weightVal << " " << p1vec[currWeightPos] << " " << dvec[currWeightPos] << " " << distance << " " << sum[currWeightPos] << endl;
                        weights.push_back(weightVal);
                    }
                    currentNetworkWeights[iter->first] = weights;
                    mapPos++;
                }
                childWeights.push_back(currentNetworkWeights);
            }
            Chromosome* child = parents[0]->clone();
            child->setWeights(childWeights);
            offspring.push_back(child);

            for(uint k = 0; k < spansize; ++k){
                delete [] initialspan[k];
                delete [] orthbasisvectors[k];
            }
            delete [] initialspan;
            delete [] orthbasisvectors;
            delete [] sum;
        }
    }

    //possibility of creating more offspring than asked for, remove until the amount is correct
    while(offspring.size() > numOffspring){
        Chromosome* toRemove = offspring[0];
        delete toRemove;
        offspring.erase(offspring.begin());
    }

    return offspring;
}