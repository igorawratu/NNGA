#ifndef ALGORITHMCREATOR_H
#define ALGORITHMCREATOR_H

#include <iostream>
#include <map>
#include <string>

#include "factory.h"
#include "mutation.h"
#include "crossover.h"
#include "selection.h"
#include "common.h"

using namespace std;

class AlgorithmCreator
{
friend class GAEngine;
typedef Selection* (*SelectionCallback)(map<string, double>);
typedef Mutation* (*MutationCallback)(map<string, double>);
typedef Crossover* (*CrossoverCallback)(map<string, double>);

public:
    static AlgorithmCreator& instance(){
        if(!initialized)
            algorithmCreator.startup();

        return algorithmCreator;
    }

    Selection* createSelectionAlgorithm(string _algorithmName, map<string, double> _parameters);
    Mutation* createMutationAlgorithm(string _algorithmName, map<string, double> _parameters);
    Crossover* createCrossoverAlgorithm(string _algorithmName, map<string, double> _parameters);

private:
    static void startup();
    static void shutdown();

    //helper functions
    void startupSelectionAlgorithms();
    void startupMutationAlgorithms();
    void startupCrossoverAlgorithms();

//disabled
private:
    AlgorithmCreator(){}
    AlgorithmCreator(const AlgorithmCreator& other){}
    const AlgorithmCreator& operator = (const AlgorithmCreator& other){return *this;}

private:
    static AlgorithmCreator algorithmCreator;
    static bool initialized;

    Factory<Selection, string, SelectionCallback, map<string, double>> mSelectionFactory;
    Factory<Crossover, string, CrossoverCallback, map<string, double>> mCrossoverFactory;
    Factory<Mutation, string, MutationCallback, map<string, double>> mMutationFactory;

};

#endif