/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Neighborhood.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/* Magic parameters for filtering the path set. */
extern const double maxPathLength = 800;
extern const double minPathPairwiseDistance = 220;

/* Magic parameters for my algorithm. */
const double radiusFactor = 0.05;

/*
 * Run two algorithms on the data.
 * Print results.
 */
void runTests (const TestData *data)
{
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *eppstein_r = epp->run();
    
    // Run my algorithm on the data
    Voss *voss = new Voss(data, radiusFactor);
    const Results *voss_r = voss->run();
    
    // Put results into a nice format
    eppstein_r->print();
    eppstein_r->saveSet();
    voss_r->print();
    voss_r->saveSet();
    delete epp;
    delete eppstein_r;
    delete voss;
    delete voss_r;
}

/*
 * Run tests on k diverse short path algorithms.
 */
int main (int, char *[])
{
    srand(1000);
    
    // Build graph to test on
    TestData data("resources/pianoHard.graphml", 10, maxPathLength, minPathPairwiseDistance);
    
    // Fix an upper limit for path length and test it
    data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
    runTests(&data);
    
    // Fix a lower limit for distance between pairs of paths and test it
    data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
    runTests(&data);
    
    return 0;
}
