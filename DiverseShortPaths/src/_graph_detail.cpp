/*
 * _graph_detail.cpp
 */

#include "_graph_detail.h"

#include "Graph.h"
#include "Neighborhood.h"

VertexAttributes::VertexAttributes ()
: state(NULL)
{
}

VertexAttributes::VertexAttributes (ompl::base::State *state)
: state(state)
{
}

heuristic::heuristic (const Graph &graph, Vertex goal)
:  g(graph), goal(goal)
{
}

double heuristic::operator() (Vertex u) const
{
    ompl::base::State *goalState = boost::get(boost::vertex_prop, g, goal).state;
    ompl::base::State *uState = boost::get(boost::vertex_prop, g, u).state;
    return g.getSpaceInfo()->distance(goalState, uState);
}

edgeWeightMap::edgeWeightMap (const Graph &graph, const std::vector<Neighborhood> &avoidThese)
:  g(graph), avoid(avoidThese)
{
}

bool edgeWeightMap::shouldAvoid (Edge e) const
{
    for (std::size_t i = 0; i < avoid.size(); i++)
    {
        if (avoid[i].shouldAvoid(e))
            return true;
    }
    return false;
}

const Graph &edgeWeightMap::getGraph () const
{
    return g;
}

namespace boost
{
    double get (const edgeWeightMap &m, const Edge &e)
    {
        if (m.shouldAvoid(e))
            return std::numeric_limits<double>::infinity();
        return m.getGraph().getEdgeWeight(e);
    }
}


visitor::visitor (Vertex goal)
: goal(goal)
{
}

void visitor::examine_vertex (Vertex u, const Graph &) const
{
    if (u == goal)
        throw foundGoalException();
}


