/*
 * Neighborhood.cpp
 */

#include "Neighborhood.h"

#include "Graph.h"

// Constructors, destructors

Neighborhood::Neighborhood (ompl::base::State *c, Edge cedge, double r)
:  center(c), centerEdge(cedge), radius(r)
{
    if (method == GRAPH)
        setupWeight();
}

Neighborhood::Neighborhood (const Neighborhood &copy)
:  center(graph->getSpaceInfo()->cloneState(copy.getCenter())), centerEdge(copy.getCenterEdge()), radius(copy.getRadius())
{
    if (method == GRAPH)
        setupWeight();
}

Neighborhood &Neighborhood::operator=(const Neighborhood &copy)
{
    Neighborhood *nbh = new Neighborhood(graph->getSpaceInfo()->cloneState(copy.center), copy.centerEdge, copy.radius);
    return *nbh;
}

Neighborhood::~Neighborhood ()
{
    // Free local resources
    graph->getSpaceInfo()->freeState(center);
}

// Public static methods

void Neighborhood::constructSharedResources (AvoidMethod m, const Graph *g)
{
    // Free previous shared resources
    if (statePool != nullptr)
        destroySharedResources();
    
    statePool = new ompl::base::State *[5];
    method = m;
    graph = g;
    
    // Allocate shared states to do computations in
    for (size_t i = 0; i < 5; i++)
    {
        statePool[i] = graph->getSpaceInfo()->allocState();
    }
}

void Neighborhood::destroySharedResources ()
{
    for (size_t i = 0; i < 5; i++)
    {
        graph->getSpaceInfo()->freeState(statePool[i]);
    }
    
    delete [] statePool;
    statePool = nullptr;
    method = UNKNOWN;
}

// Public methods

const ompl::base::State *Neighborhood::getCenter () const
{
    return center;
}

Edge Neighborhood::getCenterEdge () const
{
    return centerEdge;
}

double Neighborhood::getRadius () const
{
    return radius;
}

bool Neighborhood::shouldAvoid (Edge e) const
{
    switch (method)
    {
    case CSPACE:
        return shouldAvoid_cspace(e);
    case GRAPH:
        return shouldAvoid_graph(e);
    default:
        std::cerr << "Error: Call constructSharedResources() first!\n";
        std::exit(-1);
    }
}

// Private methods

void Neighborhood::setupWeight ()
{
    if (graph == nullptr)
    {
        std::cerr << "Error: Call constructSharedResources() first!\n";
        std::exit(-1);
    }
    
    // Compute values ahead of time that we will need often
    edgeWeight = graph->getEdgeWeight(centerEdge);
    std::tie(centerU, centerV) = graph->getVertices(centerEdge);
    centerWeight = graph->getSpaceInfo()->distance(graph->getVertexState(centerU), center);
}

bool Neighborhood::shouldAvoid_cspace (Edge e) const
{
    // Set up
    Vertex u, v;
    std::tie(u, v) = graph->getVertices(e);
    ompl::base::SpaceInformationPtr si = graph->getSpaceInfo();
    ompl::base::State *left = statePool[0];
    ompl::base::State *right = statePool[1];
    ompl::base::State *midleft = statePool[2];
    ompl::base::State *midright = statePool[3];
    ompl::base::State *mid = statePool[4];
    
    // Populate states
    si->copyState(left, graph->getVertexState(u));
    si->copyState(right, graph->getVertexState(v));
    graph->midpoint(left, right, mid);
    
    // Might be able to quit early
    if (isInside(left) || isInside(right) || isInside(mid))
        return true;
    
    // Populate more states
    graph->midpoint(left, mid, midleft);
    graph->midpoint(right, mid, midright);
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
            midleft = tmp;
            
            // If there's no chance of hitting the neighborhood, stop now
            if (left_dist > radius + h)
                return false;
        }
        else
        {
            ompl::base::State *tmp = left;
            left = mid;
            mid = midright;
            midright = tmp;
            
            // If there's no chance of hitting the neighborhood, stop now
            if (right_dist > radius + h/2)
                return false;
        }
        
        // Find new interval midpoints
        h /= 2;
        graph->midpoint(left, mid, midleft);
        graph->midpoint(right, mid, midright);
    }
    
    // We reached the neighborhood
    return true;
}

bool Neighborhood::shouldAvoid_graph(Edge e) const
{
    Vertex u, v;
    std::tie(u, v) = graph->getVertices(e);
    
    // If either endpoint is within the radius, edge should be avoided
    if (graph->graphDistance(centerU, u) + centerWeight < radius
      || graph->graphDistance(v, centerV) + edgeWeight - centerWeight < radius)
        return true;
    
    return false;
}

bool Neighborhood::isInside (const ompl::base::State *s) const
{
    return graph->getSpaceInfo()->distance(s, center) < radius;
}

// Static members

const Graph *Neighborhood::graph = nullptr;
ompl::base::State **Neighborhood::statePool = nullptr;
Neighborhood::AvoidMethod Neighborhood::method = UNKNOWN;
