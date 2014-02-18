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
    
protected:
    // Set of paths found
    std::vector<Path> pathSet;
    
    // Data to use when running the algorithm
    const TestData *testData;
    
    // Whether the last considered path was too long
    bool tooLong () const;
    
    // Whether we still need more paths
    bool needMore () const;
    
    // Evaluate a path against the set of paths found so far; return true if it is accepted
    bool considerPath(const Path &path);
    
    // Compose a Results object from the pathSet
    const Results *getResults ();
    
public:
    // Construct the algorithm to use the given data
    KDiverseShort (const TestData *data);
    
    // Clean up and destroy
    virtual ~KDiverseShort ();
    
    // Evaluate the algorithm
    virtual const Results *run () = 0;
};

#endif
