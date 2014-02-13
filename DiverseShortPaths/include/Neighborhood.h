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
    const ompl::base::State *center;
    double radius;
    
    Neighborhood (const Graph &g, const ompl::base::State *c, const double r);
    
    Neighborhood &operator=(const Neighborhood &copy);
    
    bool shouldAvoid (const Edge e) const;
    
private:
    
    bool isInside (const Vertex v) const;
};

#endif
