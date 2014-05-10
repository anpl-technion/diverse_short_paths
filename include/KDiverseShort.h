/*
 * KDiverseShort.h
 */

#ifndef __K_DIVERSE_SHORT_H
#define __K_DIVERSE_SHORT_H

#include "pch.h"

#include "Path.h"

/**
 * Abstract algorithm to find k diverse, short paths in a graph.
 */
class KDiverseShort
{
private:
    
    bool too_long;              // Whether the last considered path was too long
    Path *pathArray;            // Storage for result paths
    std::size_t i;              // Next index to store a path
    std::size_t c;              // Total paths looked at
    std::string pDistName;      // Name of the path distance measure
    
protected:
    
    /** Nearest-neighbor data structure of paths found. */
    ompl::NearestNeighbors<Path> *pathNN;
    
    /** Data to run the algorithm on. */
    const TestData *testData;
    
    /** Textual description of the algorithm. */
    std::stringstream description;
    
public:
    
    /**
     * Construct the algorithm to run on the given data set.
     * @param data  data to run on
     * @param pDist path distance measure to use
     */
    KDiverseShort (const TestData *data, PathDistanceMeasure *pDist);
    
    /** Destructor. */
    virtual ~KDiverseShort ();
    
    /**
     * Time a run of the algorithm.
     * @return collected information about the results of the run
     */
    const Results *timedRun ();
    
    /**
     * Clear internal data structures to prepare for future use.
     * @note Overriding implementations should call \c KDiverseShort::clear() in addition
     *  to performing their own clean up.
     */
    virtual void clear ();
    
protected:
    
    /**
     * Was the last considered path was too long?
     * @return true if \c considerPath() rejected path because it was too long; false otherwise
     */
    bool tooLong () const;
    
    /**
     * Do we still need more paths?
     * @return true if we have fewer paths than requested by \a testData; false otherwise
     */
    bool needMore () const;
    
    /**
     * Evaluate a path against the set of paths found so far, saving it if it meets the criteria.
     * @param path  path to evaluate
     * @return true if \a path is saved; false otherwise
     * @throw plannerTerminationException when \a stop returns true
     */
    bool considerPath(const Path &path);
    
    /**
     * Execute the algorithm and write a string to \a description.
     */
    virtual void run () = 0;
};

#endif
