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

void VertexAttributes::freeState (ompl::base::SpaceInformationPtr si)
{
    si->freeState(state);
}

const ompl::base::State *VertexAttributes::getState () const
{
    return state;
}


BOOST_CONCEPT_ASSERT((boost::AStarHeuristicConcept<heuristic, Graph>));

heuristic::heuristic (const Graph &graph, Vertex goal)
:  g(graph), goal(goal)
{
}

double heuristic::operator() (Vertex u) const
{
    const ompl::base::State *goalState = boost::get(boost::vertex_prop, g, goal).getState();
    const ompl::base::State *uState = boost::get(boost::vertex_prop, g, u).getState();
    return g.getSpaceInfo()->distance(goalState, uState);
}


BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<edgeWeightMap, Edge>));

edgeWeightMap::edgeWeightMap (const Graph &graph, const std::vector<Neighborhood> &avoidThese)
:  g(graph), avoid(avoidThese)
{
}

double edgeWeightMap::get (Edge e) const
{
    if (shouldAvoid(e))
        return std::numeric_limits<double>::infinity();
    return g.getEdgeWeight(e);
}
        
bool edgeWeightMap::shouldAvoid (Edge e) const
{
    // Check each Neighborhood to see if edge is to be avoided
    for (std::size_t i = 0; i < avoid.size(); i++)
    {
        if (avoid[i].shouldAvoid(e))
            return true;
    }
    return false;
}

namespace boost
{
    double get (const edgeWeightMap &m, const Edge &e)
    {
        return m.get(e);
    }
}


BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<visitor, Graph>));

visitor::visitor (Vertex goal)
: goal(goal)
{
}

void visitor::examine_vertex (Vertex u, const Graph &) const
{
    if (u == goal)
        throw foundGoalException();
}
