/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "KDiverseShort.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/*
 * Run tests on k diverse short path algorithms.
 */
int main (int, char *[])
{
    // Build graphs for testing
    const TestData *const data = TestData::generate();
    
    KDiverseShort *algorithm;
    
    // Run Eppstein's algorithm on the data
    algorithm = new Eppstein(data);
    const Results *eppstein_r = algorithm->run();
    delete algorithm;
    
    // Run my algorithm on the data
    algorithm = new Voss(data);
    const Results *voss_r = algorithm->run();
    delete algorithm;
    delete data;
    
    // Put results into a nice format
    Results::collate(eppstein_r, voss_r);
    delete eppstein_r;
    delete voss_r;
    
    return 0;
}
