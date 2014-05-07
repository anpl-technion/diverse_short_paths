/*
 * Neighborhood.cpp
 */

#include "Neighborhood.h"

#include "Graph.h"

Neighborhood::Neighborhood (ompl::base::State *c, Edge cedge, double r)
 :  center(c), centerEdge(cedge), radius(r)
{
    extantCount++;
    if (method == GRAPH)
        setupWeight();
}

Neighborhood::Neighborhood (const Neighborhood &copy)
 :  center(copy.g->getSpaceInfo()->cloneState(copy.center)), centerEdge(copy.centerEdge), radius(copy.radius)
{
    extantCount++;
    if (method == GRAPH)
        setupWeight();
}

Neighborhood &Neighborhood::operator=(const Neighborhood &copy)
{
    Neighborhood *nbh = new Neighborhood(copy.g->getSpaceInfo()->cloneState(copy.center), copy.centerEdge, copy.radius);
    return *nbh;
}

Neighborhood::~Neighborhood ()
{
    g->getSpaceInfo()->freeState(center);
    if (--extantCount == 0)
        destroyStatePool();
}

void Neighborhood::setupWeight ()
{
    if (g == NULL)
    {
        std::cerr << "Error: State pool not allocated!\n";
        exit(-1);
    }
    
    edgeWeight = g->getEdgeWeight(centerEdge);
    g->getVertices(centerEdge, &centerU, &centerV);
    centerWeight = g->getSpaceInfo()->distance(g->getVertexState(centerU), center);
}

void Neighborhood::setParam (AvoidMethod m)
{
    method = m;
    
    if (method == GRAPH)
        g->allPairsShortestPaths();
}

bool Neighborhood::shouldAvoid (Edge e) const
{
    if (g == NULL)
    {
        std::cerr << "Error: State pool not allocated!\n";
        exit(-1);
    }
    
    switch (method)
    {
    case CSPACE:
        return shouldAvoid_cspace(e);
    case GRAPH:
        return shouldAvoid_graph(e);
    default:
        std::cerr << "Error: Avoid Method not set!\n";
        exit(-1);
    }
}

bool Neighborhood::shouldAvoid_cspace (Edge e) const
{
    Vertex u = 0;
    Vertex v = 0;
    g->getVertices(e, &u, &v);
    
    ompl::base::SpaceInformationPtr si = g->getSpaceInfo();
    
    ompl::base::State *left = statePool[0];
    ompl::base::State *right = statePool[1];
    ompl::base::State *midleft = statePool[2];
    ompl::base::State *midright = statePool[3];
    ompl::base::State *mid = statePool[4];
    
    si->copyState(left, g->getVertexState(u));
    si->copyState(right, g->getVertexState(v));
    g->midpoint(left, right, mid);
    // Might be able to quit early
    if (isInside(left) || isInside(right) || isInside(mid))
        return true;
    
    g->midpoint(left, mid, midleft);
    g->midpoint(right, mid, midright);
    
    // h is always distance(left, mid)
    double h = si->distance(left, mid);
    double left_dist = si->distance(midleft, center);
    double right_dist = si->distance(midright, center);
    
    // Until we find we are inside the neighborhood...
    while ((left_dist = si->distance(midleft, center)) >= radius
      && (right_dist = si->distance(midright, center)) >= radius)
    {
        // Choose the closer interval
        if (left_dist < right_dist)
        {
            ompl::base::State *tmp = right;
            right = mid;
            mid = midleft;
            midleft = tmp;  // So we don't lose track of the 5 allocated states
            
            // If there's no chance of hitting the neighborhood, stop now
            if (left_dist > radius + h)
                return false;
        }
        else
        {
            ompl::base::State *tmp = left;
            left = mid;
            mid = midright;
            midright = tmp;  // So we don't lose track of the 5 allocated states
            
            // If there's no chance of hitting the neighborhood, stop now
            if (right_dist > radius + h/2)
                return false;
        }
        // Make sure h is still distance(left, mid)
        h /= 2;
        
        // Find new interval midpoints
        g->midpoint(left, mid, midleft);
        g->midpoint(right, mid, midright);
    }
    
    return true;
}

bool Neighborhood::shouldAvoid_graph(Edge e) const
{
    // If either end point is within the radius, avoid it
    Vertex u;
    Vertex v;
    g->getVertices(e, &u, &v);
    
    if (g->graphDistance(u, centerU) + centerWeight < radius
      || g->graphDistance(u, centerV) + edgeWeight - centerWeight < radius
      || g->graphDistance(v, centerU) + centerWeight < radius
      || g->graphDistance(v, centerV) + edgeWeight - centerWeight < radius)
        return true;
    
    return false;
}

void Neighborhood::allocStatePool (const Graph *graph)
{
    g = graph;
    for (size_t i = 0; i < 5; i++)
    {
        statePool[i] = g->getSpaceInfo()->allocState();
    }
}

void Neighborhood::destroyStatePool ()
{
    for (size_t i = 0; i < 5; i++)
    {
        g->getSpaceInfo()->freeState(statePool[i]);
    }
    
    delete [] statePool;
}

bool Neighborhood::isInside (const ompl::base::State *s) const
{
    return g->getSpaceInfo()->distance(s, center) < radius;
}

const Graph *Neighborhood::g = NULL;
ompl::base::State **const Neighborhood::statePool = new ompl::base::State *[5];
std::size_t Neighborhood::extantCount = 0;
Neighborhood::AvoidMethod Neighborhood::method = UNDEFINED;
