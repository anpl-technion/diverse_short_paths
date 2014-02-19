/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/* Magic parameters for filtering the path set. */
extern const double maxPathLength = 1200;
extern const double minPathPairwiseDistance = 150;

/* Magic parameters for my algorithm. */
const double radiusFactor = 0.05;
const std::size_t samplesPerPath = 20;


/*
 * Run tests on k diverse short path algorithms.
 */
int main (int, char *[])
{
    // Build graphs for testing
    const TestData *const data = TestData::generate("resources/pianoHard.graphml", 10);
    
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *eppstein_r = epp->run();
    
    // Run my algorithm on the data
    Voss *voss = new Voss(data, radiusFactor, samplesPerPath);
    const Results *voss_r = voss->run();
    
    // Put results into a nice format
    Results::collate(eppstein_r, voss_r);
    
    delete eppstein_r;
    delete voss_r;
    delete epp;
    delete voss;
    delete data;
    
    return 0;
}
