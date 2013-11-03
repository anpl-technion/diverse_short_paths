
/* Author: Caleb Voss */

#include "Graph.h"

#include "Neighborhoods.h"
#include "Path.h"

VertexAttributes::VertexAttributes ()
: state(NULL)
{
}

VertexAttributes::VertexAttributes (ompl::base::State *state)
: state(state)
{
}


Graph::Graph (const ompl::base::SpaceInformationPtr &si)
: boost_graph(), si_(si)
{
}

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

double Graph::pathLength (const Path &path) const
{
    double length = 0;
    Path::const_iterator vi = path.begin();
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


heuristic::heuristic (const Graph &graph, Vertex goal)
: g(graph), goal(goal)
{
}

double heuristic::operator() (Vertex u) const
{
    ompl::base::State *goalState = boost::get(boost::vertex_prop, g, goal).state;
    ompl::base::State *uState = boost::get(boost::vertex_prop, g, u).state;
    return g.getSpaceInformation()->distance(goalState, uState);
}


template <class N>
edgeWeightMap<N>::edgeWeightMap (const Graph &graph, Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods)
: g(graph), start(start), end(end), avoid(avoidNeighborhoods)
{
}

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

template <class N>
const Graph &edgeWeightMap<N>::getGraph (void) const
{
    return g;
}

namespace boost
{
    template <typename K, typename N>
    double get (const edgeWeightMap<N> &m, const K &e)
    {
        if (m.shouldAvoid(e))
            return std::numeric_limits<double>::max();
        return get(edge_weight, m.getGraph(), e);
    }
}


visitor::visitor (Vertex goal)
: goal(goal)
{
}

void visitor::examine_vertex (Vertex u, const Graph &g) const
{
    if (u == goal)
        throw foundGoalException();
}
