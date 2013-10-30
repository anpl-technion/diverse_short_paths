
/* Author: Caleb Voss */

#include "Graph.h"

ompl::base::SpaceInformationPtr Graph::getSpaceInformation (void) const
{
    return si_;
}

Vertex Graph::addVertex (ompl::base::State *state)
{
    return boost::add_vertex(VertexProperties(VertexPropCollection(state)), *this);
}

Edge Graph::addEdge (Vertex u, Vertex v)
{
    ompl::base::State *uState = boost::get(boost::vertex_prop, *this, u).state;
    ompl::base::State *vState = boost::get(boost::vertex_prop, *this, v).state;
    return boost::add_edge(u, v, EdgeProperties(si_->distance(uState, vState)), *this).first;
}

template <>
bool edgeWeightMap<StateSpaceNeighborhood>::distanceCheck (Vertex u, StateSpaceNeighborhood nbh) const
{
    return g.getSpaceInformation()->distance(boost::get(boost::vertex_prop, g, u).state, nbh.center) <= nbh.radius;
}
template <>
bool edgeWeightMap<GraphDistanceNeighborhood>::distanceCheck (Vertex u, GraphDistanceNeighborhood nbh) const
{
    if (nbh.radius < 0)
        return false;
    if (u == nbh.center)
        return true;
    
    BOOST_FOREACH(Vertex k, boost::adjacent_vertices(u, g))
    {
        double edgeWeight = boost::get(boost::edge_weight, g, boost::edge(u, k, g).first);
        if (distanceCheck(k, GraphDistanceNeighborhood(nbh.center, nbh.radius - edgeWeight)))
            return true;
    }
    return false;
}

template <>
bool edgeWeightMap<SingleEdgeNeighborhood>::shouldAvoid (Edge e) const
{
    Vertex u = boost::source(e, g);
    Vertex v = boost::target(e, g);
    BOOST_FOREACH(SingleEdgeNeighborhood nbh, avoid)
    {
        if (nbh.edge == e)
            return true;
    }
    return false;
}

template <>
bool edgeWeightMap<StateSpaceNeighborhood>::shouldAvoid (Edge e) const
{
    Vertex u = boost::source(e, g);
    Vertex v = boost::target(e, g);
    BOOST_FOREACH(StateSpaceNeighborhood nbh, avoid)
    {
        // Only avoid edge if one endpoint is neither start nor end, yet is inside a neighborhood
        // OR if the edge is between start and end
        if ((u == start && v == end) || (u == end && v == start))
            return true;
        if (u != start && u != end)
            if (distanceCheck(u, nbh))
                return true;
        if (v != start && v != end)
            if (distanceCheck(v, nbh))
                return true;
    }
    return false;
}

template <>
bool edgeWeightMap<GraphDistanceNeighborhood>::shouldAvoid (Edge e) const
{
    Vertex u = boost::source(e, g);
    Vertex v = boost::target(e, g);
    BOOST_FOREACH(GraphDistanceNeighborhood nbh, avoid)
    {
        // Only avoid edge if one endpoint is neither start nor end, yet is inside a neighborhood
        // OR if the edge is between start and end
        if ((u == start && v == end) || (u == end && v == start))
            return true;
        if (u != start && u != end)
            if (distanceCheck(u, nbh))
                return true;
        if (v != start && v != end)
            if (distanceCheck(v, nbh))
                return true;
    }
    return false;
}

double pathLength (const std::list<Vertex> &path, const Graph &g)
{
    double length = 0;
    std::list<Vertex>::const_iterator vi = path.begin();
    vi++;
    while (vi != path.end())
    {
        vi--;
        Vertex u = *vi;
        vi++;
        Vertex v = *vi;
        vi++;
        length += boost::get(boost::edge_weight, g, boost::edge(u, v, g).first);
    }
    return length;
}
