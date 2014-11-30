#ifndef CROSSOVERFACTORY_H
#define CROSSOVERFACTORY_H

#include <iostream>
#include <map>
#include <string>

#include "factory.h"
#include "crossover.h"
#include "common.h"

#include "multipointcrossover.h"
#include "spx.h"
#include "blx.h"
#include "undx.h"
#include "pcx.h"
#include "sbx.h"
#include "ax.h"
#include "hx.h"
#include "opx.h"
#include "tpx.h"
#include "lx.h"
#include "neatx.h"

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