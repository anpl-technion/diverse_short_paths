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
 * Run my algorithm on the data.
 * Print results.
 */
void runEppsteinTests (const TestData *data)
{
    double time;
    
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *eppstein_r = epp->timedRun(time);
    eppstein_r->saveSet();
    
    // Put results into a nice format
    eppstein_r->print(time);
    delete epp;
    delete eppstein_r;
}

/*
 * Run my algorithm on the data.
 * Print results.
 */
void runVossTests (const TestData *data, double radiusFactor)
{
    double time;
    
    // Run my algorithm on the data
    Voss *voss = new Voss(data, radiusFactor);
    const Results *voss_r = voss->timedRun(time);
    voss_r->saveSet();
    
    // Put results into a nice format
    voss_r->print(time);
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
    if (argc != 4)
    {
        std::cout << "Usage: diverse <graph> <maxLength> <minDist>\n";
        return -1;
    }
    const char *graphFile = argv[1];
    const double maxPathLength = std::atof(argv[2]);
    const double minPathPairwiseDistance = std::atof(argv[3]);
    
    // Build graph to test on
    TestData data(graphFile, 10, maxPathLength, minPathPairwiseDistance);
    
    // Eppstein
    // Fix an upper limit for path length and test it
    data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
    runEppsteinTests(&data);
    
    // Fix a lower limit for distance between pairs of paths and test it
    data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
    runEppsteinTests(&data);
    
    // Voss
    // Parameter sweep on the radiusFactor
    for (double rf = 0.005; rf <= 0.05; rf += 0.005)
    {
        // Fix an upper limit for path length and test it
        data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
        runVossTests(&data, rf);
        
        // Fix a lower limit for distance between pairs of paths and test it
        data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
        runVossTests(&data, rf);
    }
    
    return 0;
}
