/*
 * Neighborhood.h
 */

#ifndef __NEIGHBORHOOD_H
#define __NEIGHBORHOOD_H

#include "pch.h"

#include "_graph_detail.h"

struct Neighborhood
{
    const Graph &g;
    ompl::base::State **statePool;
    ompl::base::State *center;
    double radius;
    
    Neighborhood (const Graph &g, ompl::base::State **statePool, ompl::base::State *c, const double r);
    
    Neighborhood (const Neighborhood &copy);
    
    Neighborhood &operator=(const Neighborhood &copy);
    
    ~Neighborhood ();
    
    bool shouldAvoid (const Edge e) const;
    
    static ompl::base::State **allocStatePool (const Graph &g);
    
    static void destroyStatePool (const Graph &g, ompl::base::State **statePool);
    
private:
    
    bool isInside (const ompl::base::State *s) const;
};

#endif
