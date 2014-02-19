/*
 * Results.h
 */

#ifndef __RESULTS_H
#define __RESULTS_H

#include "pch.h"

class Path;
class TestData;

/*
 * Representation of experiment results for an algorithm.
 */
class Results
{
private:
    const char *algorithm;
    const TestData *const data;
    std::vector<Path *> paths;
    
public:
    
    // Constructor
    Results (const char *alg_name, const TestData *const testData, std::vector<Path> &pathSet);
    
    // Print self
    void print (std::size_t i) const;
    
    // Combine sets of results into pretty format
    static void collate (const Results *r1, const Results *r2);
    
    double findShortestLength () const;
    
    double findLongestLength () const;
    
    double meanNearestPathDistance () const;
    
    double nearestPathDistance (const std::size_t which) const;
    
    // Get test data
    const TestData *getData () const;
    
    // Get the number of paths
    std::size_t numPaths () const;
    
    // Get a path
    const Path *getPath (std::size_t i) const;
};

#endif
