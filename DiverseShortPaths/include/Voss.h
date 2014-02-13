/*
 * Voss.h
 */

#ifndef __VOSS_H
#define __VOSS_H

#include "pch.h"

#include "KDiverseShort.h"

struct Neighborhood;
class Path;
class TestData;

/*
 * My algorithm for k short, diverse paths by stochastic avoidance.
 */
class Voss : public KDiverseShort
{
public:
    // Construct the algorithm to use the given data
    Voss (const TestData *data);
    
    // Evaluate the algorithm
    const Results *run ();
    
private:
    // Compute the shortest path under avoidance constraints
    Path getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoidThese) const;
    
    // Randomly sample a point on the path
    ompl::base::State *sampleFromPath (const Path &path) const;
};

#endif
