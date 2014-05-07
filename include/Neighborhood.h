/*
 * Neighborhood.h
 */

#ifndef __NEIGHBORHOOD_H
#define __NEIGHBORHOOD_H

#include "pch.h"

#include "_graph_detail.h"

struct Neighborhood
{
    enum AvoidMethod { UNDEFINED, CSPACE, GRAPH };
    
    static const Graph *g;
    static ompl::base::State **const statePool;
    static std::size_t extantCount;
    static AvoidMethod method;
    
    ompl::base::State *center;
    Edge centerEdge;
    double edgeWeight;
    double centerWeight;
    Vertex centerU;
    Vertex centerV;
    
    double radius;
    
    Neighborhood (ompl::base::State *c, Edge cedge, double r);
    
    Neighborhood (const Neighborhood &copy);
    
    Neighborhood &operator=(const Neighborhood &copy);
    
    ~Neighborhood ();
    
    void setupWeight ();
    
    static void setParam (AvoidMethod m);
    
    bool shouldAvoid (Edge e) const;
    
    static void allocStatePool (const Graph *graph);
    
    static void destroyStatePool ();
    
private:
    
    bool shouldAvoid_cspace (Edge e) const;
    
    bool shouldAvoid_graph (Edge e) const;
    
    bool isInside (const ompl::base::State *s) const;
};

#endif
