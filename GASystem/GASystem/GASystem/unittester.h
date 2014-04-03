#ifndef UNITTESTER_H
#define UNITTESTER_H

#include <vector>

using namespace std;

class UnitTester// : public CppUnit::TestCase
{
public:
    void runTest();

private:
    void testLoadStore();
    void testANNSerialization();
    void testSolutionSerialization();
    void testLoadStoreFixed();
    void testSigmoidOutput();
    void testHidden();
    void testLoop();
    void testCopyAssignment();
    void testWeightsAndStructure();
    void testHiddenSigmoidOutput();
    void testSolutionLoadWrite();
    void testSolutionEvaluation();
    void testChromosomeLoad();
    void testChromosomeCopyAss();
    void testChromosomeGetSet();
    void testGaussianMutation();
    void testRankSelection();
    void testFactories();
    void testMultipointCrossover();
    void testGA();

};

#endif