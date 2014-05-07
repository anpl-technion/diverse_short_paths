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
    const std::string description;
    const TestData *const data;
    const Path *paths;
    const std::size_t n;
    
public:
    
    // Constructor
    Results (const std::string name, const TestData *const testData,
             const Path *pathSet, const std::size_t nPaths);
    
    // Write set in OMPL format
    void saveSet () const;
    
    // Print self
    void print (double runtime) const;
    
    double findShortestLength () const;
    
    double findLongestLength () const;
    
    double diversity () const;
    
    double minNearestPathDistance () const;
    
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
