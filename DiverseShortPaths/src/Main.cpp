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
extern const double maxPathLength = 4000;
extern const double minPathPairwiseDistance = 0;

/* Magic parameters for my algorithm. */
const double radiusFactor = 0.05;
const std::size_t samplesPerPath = 20;

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
    Voss *voss = new Voss(data, radiusFactor, samplesPerPath);
    const Results *voss_r = voss->run();
    
    // Put results into a nice format
    eppstein_r->print();
    voss_r->print();
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
    srand(time(NULL));
    
    // Build graph to test on
    TestData data("resources/cubiclePRM.graphml", 10, maxPathLength, minPathPairwiseDistance);
    
    // Fix an upper limit for path length and test it
    data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
    std::cout << "Fixing maximum path length at " << maxPathLength << "...\n\n";
    runTests(&data);
    
    // Fix a lower limit for distance between pairs of paths and test it
    data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
    std::cout << "Fixing minimum distance between paths at " << minPathPairwiseDistance << "...\n\n";
    runTests(&data);
    
    return 0;
}
