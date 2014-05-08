/*
 * Results.h
 */

#ifndef __RESULTS_H
#define __RESULTS_H

#include "pch.h"

/**
 * Results of running an algorithm.
 */
class Results
{
private:
    
    const std::string description;  // Textual description of the algorithm
    const TestData *const data;     // Data algorithm ran on
    const Path *paths;              // Array of paths found by the algorithm
    const std::size_t n;            // Number of paths found
    const double t;                 // Execution time
    
public:
    
    /**
     * Construct results from all the details.
     * @param desc      description of the algorithm
     * @param testData  input to the algorithm
     * @param pathSet   array of paths found
     * @param nPaths    number of paths found
     * @param runtime   time taken to execute algorithm
     */
    Results (const std::string &desc, const TestData *testData, const Path *pathSet, std::size_t nPaths, double runtime);
    
    /** Write all paths to the same file in OMPL App format. */
    void saveSet () const;
    
    /** Print a summary of these results. */
    void print () const;
    
    /**
     * Get the execution time of the algorithm.
     * @return our execution time \a t
     */
    double getTime () const;
    
    /**
     * Get the length of the shortest of our paths.
     * @return length of shortest path in our \a paths set
     * @warning Assumes this is actually just the first path due to underlying algorithms.
     */
    double findShortestLength () const;
    
    /**
     * Get the length of the longest of our paths.
     * @return length of longest path in our \a paths set
     */
    double findLongestLength () const;
    
    /**
     * Get the minimum distance between any pair of paths.
     * @return min of distances between every pair of distance
     *  paths in our \a paths set
     */
    double minNearestPathDistance () const;
    
    /**
     * Get the average distance between any pair of paths.
     * @return mean of distances between every path and its nearest
     *  neighboring path in our \a paths set
     */
    double meanNearestPathDistance () const;
    
    /**
     * Get the distance to the nearest neighbor of some path.
     * @param which index of path to find nearest neighbor for
     * @return distance to nearest neighbor of path \a which
     * @warning Does not check bounds on \a which.
     */
    double nearestPathDistance (const std::size_t which) const;
    
    /**
     * Get the number of paths.
     * @return our \a nPaths
     */
    std::size_t numPaths () const;
};

#endif
