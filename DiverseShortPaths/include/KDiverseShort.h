/*
 * KDiverseShort.h
 */

#ifndef __K_DIVERSE_SHORT_H
#define __K_DIVERSE_SHORT_H

#include "pch.h"

class Path;
class Results;
class TestData;

/*
 * Abstract algorithm to find k diverse, short paths in a graph.
 */
class KDiverseShort
{
private:
    // Whether the last considered path was too long
    bool too_long;
    
    // Storage for paths
    Path *pathArray;
    
    // Next index to store a path
    std::size_t _i;
    
    // Total paths looked at
    std::size_t c;
    
protected:
    // Set of paths found
    ompl::NearestNeighbors<Path> *pathNN;
    
    // Data to use when running the algorithm
    const TestData *testData;
    
    // Whether the last considered path was too long
    bool tooLong () const;
    
    // Whether we still need more paths
    bool needMore () const;
    
    // Evaluate a path against the set of paths found so far; return true if it is accepted
    bool considerPath(const Path &path);
    
    // Compose a Results object from the pathNN
    const Results *getResults (const char *alg_name);
    
    // Evaluate the algorithm
    virtual const Results *run () = 0;
    
public:
    // Construct the algorithm to use the given data
    KDiverseShort (const TestData *data);
    
    // Clean up and destroy
    virtual ~KDiverseShort ();
    
    // Time a run of the algorithm
    const Results *timedRun (double &seconds);
    
};

#endif
