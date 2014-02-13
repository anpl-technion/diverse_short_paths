/*
 * Neighborhood.cpp
 */

#include "Neighborhood.h"

#include "Graph.h"

Neighborhood::Neighborhood (const Graph &g, const ompl::base::State *c, const double r)
: g(g), center(c), radius(r)
{
}

Neighborhood &Neighborhood::operator=(const Neighborhood &copy)
{
    Neighborhood *nbh = new Neighborhood(copy.g, copy.center, copy.radius);
    return *nbh;
}

bool Neighborhood::shouldAvoid (const Edge e) const
{
    Vertex u = 0;
    Vertex v = 0;
    g.getVertices(e, &u, &v);
    return isInside(u) || isInside(v);
}

bool Neighborhood::isInside (const Vertex v) const
{
    return g.getSpaceInfo()->distance(g.getVertexState(v), center) <= radius;
}
