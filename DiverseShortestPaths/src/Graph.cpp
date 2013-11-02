
/* Author: Caleb Voss */

#include "Graph.h"
#include "Path.h"

ompl::base::SpaceInformationPtr Graph::getSpaceInformation (void) const
{
    return si_;
}

Vertex Graph::addVertex (ompl::base::State *state)
{
    return boost::add_vertex(VertexProperty(VertexAttributes(state)), *this);
}

Edge Graph::addEdge (Vertex u, Vertex v)
{
    ompl::base::State *uState = boost::get(boost::vertex_prop, *this, u).state;
    ompl::base::State *vState = boost::get(boost::vertex_prop, *this, v).state;
    return boost::add_edge(u, v, EdgeProperty(si_->distance(uState, vState)), *this).first;
}

double Graph::pathLength (const std::list<Vertex> &path) const
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
        length += boost::get(boost::edge_weight, *this, boost::edge(u, v, *this).first);
    }
    return length;
}

template <class N>
Path Graph::getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods) const
{
    // Run the A* search
    std::vector<Vertex> pred(boost::num_vertices(*this));
    std::list<Vertex> path;
    try
    {
        boost::astar_search(*this, start, heuristic(*this, end),
                            boost::weight_map(edgeWeightMap<N>(*this, start, end, avoidNeighborhoods)).
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
    return Path(path, *this);
}
// Needed for linking, though this locks us in to using one of these three neighborhoods
template Path Graph::getShortestPathWithAvoidance<StateSpaceNeighborhood> (Vertex start, Vertex end, const std::vector<StateSpaceNeighborhood> &avoidNeighborhoods) const;
template Path Graph::getShortestPathWithAvoidance<GraphDistanceNeighborhood> (Vertex start, Vertex end, const std::vector<GraphDistanceNeighborhood> &avoidNeighborhoods) const;
template Path Graph::getShortestPathWithAvoidance<SingleEdgeNeighborhood> (Vertex start, Vertex end, const std::vector<SingleEdgeNeighborhood> &avoidNeighborhoods) const;

template <class N>
bool edgeWeightMap<N>::shouldAvoid (Edge e) const
{
    BOOST_FOREACH(N nbh, avoid)
    {
        if (nbh.shouldAvoid(e))
            return true;
    }
    return false;
}

