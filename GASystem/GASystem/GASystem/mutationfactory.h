#ifndef MUTATIONFACTORY_H
#define MUTATIONFACTORY_H

#include <iostream>
#include <map>
#include <string>

#include "factory.h"
#include "common.h"
#include "mutation.h"

#include "gaussianmutation.h"
#include "cauchylorentzmutation.h"
#include "uniformmutation.h"

using namespace std;

class MutationFactory
{
friend class GAEngine;
typedef Mutation* (*MutationCallback)();

public:
    static MutationFactory& instance();

    Mutation* create(string _algorithmName);

private:
    static void startup();
    static void shutdown();

//disabled
private:
    MutationFactory(){}
    MutationFactory(const MutationFactory& other){}
    const MutationFactory& operator = (const MutationFactory& other){return *this;}
    ~MutationFactory(){}

private:
    static MutationFactory mutationFactory;
    static bool initialized;

    Factory<Mutation, string, MutationCallback> mFactory;
};


#endif