/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Neighborhood.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/*
 * Run two algorithms on the data.
 * Print results.
 */
void runTests (const TestData *data, double radiusFactor)
{
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *eppstein_r = epp->run();
    eppstein_r->saveSet();
    
    // Run my algorithm on the data
    Voss *voss = new Voss(data, radiusFactor);
    const Results *voss_r = voss->run();
    voss_r->saveSet();
    
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
int main (int argc, char *argv[])
{
    srand(1000);
    
    // Parse command line args
    if (argc != 5)
    {
        std::cout << "Usage: diverse <graph> <maxLength> <minDist> <radiusFactor>\n";
        return -1;
    }
    const char *graphFile = argv[1];
    const double maxPathLength = std::atof(argv[2]);
    const double minPathPairwiseDistance = std::atof(argv[3]);
    const double radiusFactor = std::atof(argv[4]);
    
    // Build graph to test on
    TestData data(graphFile, 10, maxPathLength, minPathPairwiseDistance);
    
    // Fix an upper limit for path length and test it
    data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
    runTests(&data, radiusFactor);
    
    // Fix a lower limit for distance between pairs of paths and test it
    data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
    runTests(&data, radiusFactor);
    
    return 0;
}
