
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

std::list<Vertex> Graph::getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<Neighborhood> &avoidNeighborhoods) const
{
    // Run the A* search
    std::vector<Vertex> pred(boost::num_vertices(*this));
    std::list<Vertex> path;
    try
    {
        boost::astar_search(*this, start, heuristic(*this, end),
                            boost::weight_map(edgeWeightMap(*this, start, end, avoidNeighborhoods)).
                            predecessor_map(&pred[0]).
                            visitor(visitor(end)));
    }
    catch (foundGoalException e)
    {
        for (Vertex v = end;; v = pred[v])
        {
            path.push_front(v);
            if (pred[v] == v)
                break;
        }
    }
    
    if (path.size() == 1 && start != end)
        path.clear();
    return path;
}
