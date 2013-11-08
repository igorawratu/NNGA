#include "undx.h"


UNDX::UNDX(){}
UNDX::~UNDX(){}

vector<Chromosome*> UNDX::execute(vector<Chromosome*> _population, uint numOffspring, map<string, double>& _parameters){
    Selection* selectionAlgorithm = SelectionFactory::instance().create("QuadraticRankSelection");
    if(!selectionAlgorithm)
        return _population;
    
    vector<Chromosome*> offspring;

    while(offspring.size() < numOffspring){
        vector<Chromosome*> parents = selectionAlgorithm->execute(_population, 3, vector<Chromosome*>());

        vector<map<uint, vector<double>>> p1Weights, p2Weights, p3Weights, childWeights;
        double dp3distance = 0;
        vector<double> mvec, dvec, p1vec, p2vec, p3vec;

        p1Weights = parents[0]->getWeightData(); p2Weights = parents[1]->getWeightData(); p3Weights = parents[2]->getWeightData();
        
        //calculate line d(distance vector from p1 to p2) and m(midpoint of p1 and p2), also put other stuff in normal vector to make things easier to code
        for(uint k = 0; k < p1Weights.size(); k++){
            for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                for(uint i = 0; i < iter->second.size(); i++){
                    dvec.push_back(p1Weights[k][iter->first][i] - p2Weights[k][iter->first][i]);
                    mvec.push_back((p1Weights[k][iter->first][i] + p2Weights[k][iter->first][i]) / 2);
                    p1vec.push_back(p1Weights[k][iter->first][i]);
                    p2vec.push_back(p2Weights[k][iter->first][i]);
                    p3vec.push_back(p3Weights[k][iter->first][i]);
                }
            }
        }

        //calculate projection of p3 onto line d, and then calc the distance between the projection and p3
        double dDotp3 = 0, dDotd = 0;
        dp3distance = 0;
        for(uint k = 0; k < p3vec.size(); ++k){
            dDotp3 += p3vec[k] * dvec[k];
            dDotd += dvec[k] * dvec[k];
        }

        for(uint k = 0; k < p3vec.size(); ++k){
            double projpoint = dvec[k]* (dDotp3/dDotd);
            dp3distance += (projpoint - p3vec[k]) * (projpoint - p3vec[k]);
        }

        sqrt(dp3distance);


        //calculate the span of the subspace perpendicular to d
        const int spansize = dvec.size() - 1;
        const int spanvecdim = dvec.size();
        double **initialspan = new double*[spansize];
        double **orthbasisvectors = new double*[spansize];      
        double *sum = new double[spanvecdim];

        for(uint k = 0; k < spanvecdim; ++k)
            sum[k] = 0;


        for(uint k = 0; k < spansize; ++k){
            initialspan[k] = new double[spanvecdim];
            orthbasisvectors[k] = new double[spanvecdim];

            for(uint i = 0; i < spanvecdim; ++i){
                initialspan[k][i] = 0;
                orthbasisvectors[k][i] = 0;
            }
        }

        for(uint k = 0; k < spanvecdim; ++k){
            if(!k)
                continue;

            initialspan[k - 1][0] = dvec[k] / -dvec[0];
            initialspan[k - 1][k] = 1;
        }

        //run Gram-Schmidt Process to find orthonormal basis of subspace defined by span
        for(uint k = 0; k < spansize; ++k){
            for(uint i = 0; i < spanvecdim; ++i)
                orthbasisvectors[k][i] = initialspan[k][i];

            for(uint i = 0; i < k; ++i){
                double pointDot = 0, lineDot = 0;
                for(uint j = 0; j < spanvecdim; ++j){
                    pointDot += orthbasisvectors[i][j] * orthbasisvectors[k][j];
                    lineDot += orthbasisvectors[i][j] * orthbasisvectors[i][j];
                }

                for(uint j = 0; j < spanvecdim; ++j)
                    orthbasisvectors[k][j] -= orthbasisvectors[i][j] * pointDot/lineDot;
            }
        }

        //normalise the orthogonal vectors
        for(uint k = 0; k < spansize; ++k){
            double distance = 0;
            for(uint i = 0; i < spanvecdim; ++i)
                distance += orthbasisvectors[k][i] * orthbasisvectors[k][i];
            sqrt(distance);

            for(uint i = 0; i < spanvecdim; ++i)
                orthbasisvectors[k][i] /= distance;
        }

        //rngs
        double sigone = 0.35/sqrt((double)dvec.size()) * 0.35/sqrt((double)dvec.size());
        double sigtwo = 0.25;

        boost::mt19937 rng(rand());
        boost::normal_distribution<> dist(0, sigone);
        boost::variate_generator<boost::mt19937, boost::normal_distribution<double>> n1(rng, dist);

        boost::mt19937 rng2(rand());
        boost::normal_distribution<> dist2(0, sigtwo);
        boost::variate_generator<boost::mt19937, boost::normal_distribution<double>> n2(rng2, dist2);


        //calculate sum and rng
        for(uint k = 0; k < spansize; ++k){
            double rval = n2();

            for(uint i = 0; i < spanvecdim; ++i)
                sum[k] += orthbasisvectors[k][i] * rval;
        }


        //run the actual undx equation
        for(uint k = 0; k < p1Weights.size(); k++){
            map<uint, vector<double>> currentNetworkWeights;
            uint mapPos = 0;
            for(map<uint, vector<double>>::iterator iter = p1Weights[k].begin(); iter != p1Weights[k].end(); iter++){
                vector<double> weights;
                for(uint i = 0; i < iter->second.size(); i++){
                    uint currWeightPos = k+mapPos+i;
                    double weightVal = mvec[currWeightPos] + dvec[currWeightPos] * n1() + dp3distance * sum[currWeightPos];
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
    delete selectionAlgorithm;

    return offspring;
}