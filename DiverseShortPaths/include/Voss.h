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
private:
    static const char *VOSS_NAME;
    
    const double radius_factor;
    const std::size_t samples_per_path;
    
public:
    // Construct the algorithm to use the given data
    Voss (const TestData *data, double radiusFactor, std::size_t samplesPerPath);
    
    // Evaluate the algorithm
    const Results *run ();
    
private:
    // Compute the shortest path under avoidance constraints
    Path getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoidThese) const;
    
    // Randomly sample a point on the path
    ompl::base::State *sampleFromPath (const Path &path) const;
};

#endif
