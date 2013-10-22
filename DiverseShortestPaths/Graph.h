
/* Author: Caleb Voss */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

namespace boost {
    enum vertex_prop_t { vertex_prop };
    BOOST_INSTALL_PROPERTY(vertex, prop);
}

struct VertexPropCollection;

typedef boost::adjacency_list <
    boost::vecS,boost::vecS,boost::undirectedS,
    boost::property <boost::vertex_prop_t,VertexPropCollection>,
    boost::property <boost::edge_weight_t,double> > base_graph;
typedef boost::graph_traits<base_graph> graph_traits;
typedef graph_traits::vertex_descriptor Vertex;
typedef graph_traits::vertex_iterator VertexIterator;
typedef graph_traits::edge_descriptor Edge;
typedef boost::property <boost::vertex_prop_t,VertexPropCollection> VertexProperties;
typedef boost::property <boost::edge_weight_t,double> EdgeProperties;

struct StateSpaceNeighborhood
{
    typedef ompl::base::State *center_type;
    typedef double radius_type;
    
    center_type center;
    radius_type radius;
    
    StateSpaceNeighborhood (center_type c, radius_type r)
    : center(c), radius(r)
    {
    }
};

struct GraphDistanceNeighborhood
{
    typedef Vertex center_type;
    typedef unsigned int radius_type;
    
    center_type center;
    radius_type radius;
    
    GraphDistanceNeighborhood (center_type c, radius_type r)
    : center(c), radius(r)
    {
    }
};

struct VertexPropCollection
{
    ompl::base::State *state;
    
    VertexPropCollection ()
    : state(NULL)
    {
    }
    
    VertexPropCollection (ompl::base::State *state)
    : state(state)
    {
    }
};

class Graph : public base_graph
{
private:
    
    ompl::base::SpaceInformationPtr si_;
    
public:
    
    /** \brief Initialize an empty graph */
    Graph (const ompl::base::SpaceInformationPtr &si)
    : base_graph(), si_(si)
    {
    }
    
    /** \brief Get this graph's SpaceInformationPtr
     * 
     * @return          our SpaceInformationPtr
     */
    ompl::base::SpaceInformationPtr getSpaceInformation (void) const;
    
    /** \brief Add a vertex to this graph with a specified state
     * 
     * @param state     state associated with this vertex
     * 
     * @return          vertex descriptor of the new vertex
     */
    Vertex addVertex (ompl::base::State *state);
    
    /** \brief Add an edge to this graph between two vertices, with an edge weight
     *         equal to the StateSpace distance between the vertices' states
     * 
     * @param u     first vertex
     * @param v     second vertex
     * 
     * @return      edge descriptor of the new edge
     */
    Edge addEdge (Vertex u, Vertex v);
        
    /** \brief Find the shortest path between two vertices, honoring vertex avoidance
     * 
     * @tparam N        Neighborhood class to use
     * 
     * @param start     starting vertex
     * @param end       ending vertex
     * 
     * @return          list of the vertices in the path from start to end; empty list if no path was found
     */
    template <class N>
    std::list<Vertex> getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods) const;
    
};

class heuristic // implements AStarHeuristic
{
    const Graph &g;
    const Vertex goal;
    
public:
    heuristic (const Graph &graph, Vertex goal)
    : g(graph), goal(goal)
    {
    }
    
    double operator() (Vertex u) const
    {
        ompl::base::State *goalState = boost::get(boost::vertex_prop, g, goal).state;
        ompl::base::State *uState = boost::get(boost::vertex_prop, g, u).state;
        return g.getSpaceInformation()->distance(goalState, uState);
    }
};

template <class N>
class edgeWeightMap // implements ReadablePropertyMap
{
    const Graph &g;
    const Vertex start;
    const Vertex end;
    const std::vector<N> &avoid;
    
    // Return whether distance between u and v is <= max
    bool distanceCheck (Vertex u, typename N::center_type v, typename N::radius_type max) const;
    
public:
    
    typedef Edge key_type;
    typedef double value_type;
    typedef double reference;
    typedef boost::readable_property_map_tag category;
    
    edgeWeightMap (const Graph &graph, Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods)
    : g(graph), start(start), end(end), avoid(avoidNeighborhoods)
    {
    }
    
    bool shouldAvoid (Edge e) const
    {
        Vertex u = boost::source(e, g);
        Vertex v = boost::target(e, g);
        BOOST_FOREACH(N nbh, avoid)
        {
            // Only avoid edge if one endpoint is neither start nor end, yet is inside a neighborhood
            // OR if the edge is between start and end
            if ((u == start && v == end) || (u == end && v == start))
                return true;
            if (u != start && u != end)
                if (distanceCheck(u, nbh.center, nbh.radius))
                    return true;
            if (v != start && v != end)
                if (distanceCheck(v, nbh.center, nbh.radius))
                    return true;
        }
        return false;
    }
    
    const base_graph &getGraph (void) const
    {
        return g;
    }
};

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

struct foundGoalException {};
class visitor : public boost::default_astar_visitor // implements AStar_Visitor
{
    Vertex goal;
    
public:
    visitor (Vertex goal)
    : goal(goal)
    {
    }
    
    void examine_vertex(Vertex u, const Graph &g) const
    {
        if (u == goal)
            throw foundGoalException();
    }
};

template <class N>
std::list<Vertex> Graph::getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods) const
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
    return path;
}

#endif
