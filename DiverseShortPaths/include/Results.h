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
    std::string description;
    const TestData *const data;
    std::vector<Path *> paths;
    
public:
    
    // Constructor
    Results (std::string name, const TestData *const testData, std::vector<Path> &pathSet);
    
    // Write set in OMPL format
    void saveSet () const;
    
    // Print self
    void print (double runtime) const;
    
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
