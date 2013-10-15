
/* Author: Caleb Voss */

#include "Graph.h"

std::list<Vertex> Graph::getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<Neighborhood> &avoidNeighborhoods) const
{
    // Run the A* search
    std::vector<Vertex> pred(boost::num_vertices(*this));
    std::list<Vertex> path;
    try
    {
        boost::astar_search(*this, start, heuristic(*this, end),
                            boost::weight_map(edgeWeightMap(*this, avoidNeighborhoods)).
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
