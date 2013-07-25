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
typedef Selection* (*SelectionCallback)();
typedef Mutation* (*MutationCallback)();
typedef Crossover* (*CrossoverCallback)();

public:
    static AlgorithmCreator& instance(){
        if(!initialized)
            algorithmCreator.startup();

        return algorithmCreator;
    }

    Selection* createSelectionAlgorithm(string _algorithmName);
    Mutation* createMutationAlgorithm(string _algorithmName);
    Crossover* createCrossoverAlgorithm(string _algorithmName);

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

    Factory<Selection, string, SelectionCallback> mSelectionFactory;
    Factory<Crossover, string, CrossoverCallback> mCrossoverFactory;
    Factory<Mutation, string, MutationCallback> mMutationFactory;

};

#endif