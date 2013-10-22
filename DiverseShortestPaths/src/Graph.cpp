
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
bool edgeWeightMap<Neighborhood>::distanceCheck (Vertex u, ompl::base::State *v, double max) const
{
    return g.getSpaceInformation()->distance(boost::get(boost::vertex_prop, g, u).state, v) <= max;
}
template <>
bool edgeWeightMap<Neighborhood2>::distanceCheck (Vertex u, Vertex v, unsigned int max) const
{
    if (max == 0)
        return u==v;
    
    BOOST_FOREACH(Vertex k, boost::adjacent_vertices(u, g))
    {
        if (distanceCheck(k, v, max-1))
            return true;
    }
    return false;
}
