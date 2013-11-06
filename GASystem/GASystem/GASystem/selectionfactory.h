#ifndef SELECTIONFACTORY_H
#define SELECTIONFACTORY_H

#include <iostream>
#include <map>
#include <string>

#include "factory.h"
#include "common.h"
#include "selection.h"

#include "nlrankselection.h"
#include "lrankselection.h"
#include "quadraticrankselection.h"
#include "tournamentselection.h"
#include "boltzmannselection.h"

using namespace std;

class SelectionFactory
{
friend class GAEngine;
typedef Selection* (*SelectionCallback)();

public:
    static SelectionFactory& instance();

    Selection* create(string _algorithmName);

private:
    static void startup();
    static void shutdown();

//disabled
private:
    SelectionFactory(){}
    SelectionFactory(const SelectionFactory& other){}
    const SelectionFactory& operator = (const SelectionFactory& other){return *this;}
    ~SelectionFactory(){}

private:
    static SelectionFactory selectionFactory;
    static bool initialized;

    Factory<Selection, string, SelectionCallback> mFactory;
};

#endif