#ifndef CROSSOVERFACTORY_H
#define CROSSOVERFACTORY_H

#include <iostream>
#include <map>
#include <string>

#include "factory.h"
#include "crossover.h"
#include "common.h"

#include "multipointcrossover.h"

using namespace std;

class CrossoverFactory
{
friend class GAEngine;
typedef Crossover* (*CrossoverCallback)();

public:
    static CrossoverFactory& instance();

    Crossover* create(string _algorithmName);

private:
    static void startup();

    static void shutdown();

//disabled
private:
    CrossoverFactory(){}
    CrossoverFactory(const CrossoverFactory& other){}
    const CrossoverFactory& operator = (const CrossoverFactory& other){return *this;}
    ~CrossoverFactory(){}

private:
    static CrossoverFactory crossoverFactory;
    static bool initialized;

    Factory<Crossover, string, CrossoverCallback> mFactory;

};

#endif