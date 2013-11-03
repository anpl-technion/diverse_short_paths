
/* Author: Caleb Voss */

#ifndef NEIGHBORHOODS_H
#define NEIGHBORHOODS_H

#include <ompl/base/State.h>

#include "Common.h"

struct StateSpaceNeighborhood
{
    typedef ompl::base::State *center_type;
    typedef double radius_type;
    
    const Graph &g;
    center_type center;
    radius_type radius;
    
    StateSpaceNeighborhood (const Graph &g, const center_type c, const radius_type r);
    
    StateSpaceNeighborhood &operator=(const StateSpaceNeighborhood &copy);
    
    bool shouldAvoid (const Edge e) const;
    
private:
    
    bool isInside (const Vertex u) const;
};

struct GraphDistanceNeighborhood
{
    typedef Vertex center_type;
    typedef double radius_type;
    
    const Graph &g;
    center_type center;
    radius_type radius;
    
    GraphDistanceNeighborhood (const Graph &g, const center_type c, const radius_type r);
    
    GraphDistanceNeighborhood &operator=(const GraphDistanceNeighborhood &copy);
    
    bool shouldAvoid (const Edge e) const;
    
private:
    
    bool isInside (const Vertex u, const radius_type r) const;
};

struct SingleEdgeNeighborhood
{
    Edge edge;
    
    SingleEdgeNeighborhood (const Edge e);
    
    bool shouldAvoid (const Edge e) const;
};

#endif
