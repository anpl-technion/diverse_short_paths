
/* Author: Caleb Voss */

#include "Neighborhoods.h"

#include "Graph.h"

StateSpaceNeighborhood::StateSpaceNeighborhood (const Graph &g, const center_type c, const radius_type r)
: g(g), center(c), radius(r)
{
}

StateSpaceNeighborhood &StateSpaceNeighborhood::operator=(const StateSpaceNeighborhood &copy)
{
    StateSpaceNeighborhood *nbh = new StateSpaceNeighborhood(copy.g, copy.center, copy.radius);
    return *nbh;
}

bool StateSpaceNeighborhood::shouldAvoid (const Edge e) const
{
    const Vertex u = boost::source(e, g);
    const Vertex v = boost::target(e, g);
    return isInside(u) || isInside(v);
}

bool StateSpaceNeighborhood::isInside (const Vertex u) const
{
    return g.getSpaceInformation()->distance(boost::get(boost::vertex_prop, g, u).state, center) <= radius;
}


GraphDistanceNeighborhood::GraphDistanceNeighborhood (const Graph &g, const center_type c, const radius_type r)
: g(g), center(c), radius(r)
{
}

GraphDistanceNeighborhood &GraphDistanceNeighborhood::operator=(const GraphDistanceNeighborhood &copy)
{
    GraphDistanceNeighborhood *nbh = new GraphDistanceNeighborhood(copy.g, copy.center, copy.radius);
    return *nbh;
}

bool GraphDistanceNeighborhood::shouldAvoid (const Edge e) const
{
    Vertex u = boost::source(e, g);
    Vertex v = boost::target(e, g);
    return isInside(u, radius) || isInside(v, radius);
}

// !!! This is really slow
bool GraphDistanceNeighborhood::isInside (const Vertex u, const radius_type r) const
{
    if (r < 0)
        return false;
    if (u == center)
        return true;
    
    BOOST_FOREACH(const Edge e, boost::out_edges(u, g))
    {
        double edgeWeight = boost::get(boost::edge_weight, g, e);
        if (isInside(boost::target(e, g), r - edgeWeight))
            return true;
    }
    return false;
}


SingleEdgeNeighborhood::SingleEdgeNeighborhood (const Edge e)
: edge(e)
{
}

bool SingleEdgeNeighborhood::shouldAvoid (const Edge e) const
{
    return e == edge;
}
