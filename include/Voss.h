/*
 * Voss.h
 */

#ifndef __VOSS_H
#define __VOSS_H

#include "pch.h"

#include "KDiverseShort.h"

/**
 * My algorithm for k short, diverse paths by random avoidance.
 */
class Voss : public KDiverseShort
{
private:
    
    const double radiusFactor;  // Size of avoided regions as a factor of shortest path length
    
public:
    
    /**
     * Construct an instance of this algorithm.
     * @param data          data set to run on
     * @param radiusFactor  radius of avoided regions as a factor of shortest path length
     */
    Voss (const TestData *data, double radiusFactor);
    
private:
    
    /**
     * Execute the algorithm.
     * @return textual description of the algorithm
     */
    std::string run ();
    
    /**
     * Compute the shortest path under avoidance constraints.
     * @param avoidThese    set of regions paths are now forbidden from entering
     * @return shortest path through the graph that does not enter a forbidden region,
     *  or an empty Path object if no such path exists
     */
    Path getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoidThese) const;
};

#endif
