
/* Author: Caleb Voss */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include "Common.h"

namespace boost {
    BOOST_INSTALL_PROPERTY(vertex, prop);
}

// Collection of attributes for a vertex
struct VertexAttributes
{
    // Vertex's location in the state space in which the graph is embedded
    ompl::base::State *state;
    
    VertexAttributes ()
    : state(NULL)
    {
    }
    
    VertexAttributes (ompl::base::State *state)
    : state(state)
    {
    }
};

class Graph : public boost_graph
{
private:
    
    ompl::base::SpaceInformationPtr si_;
    
public:
    
    /** \brief Initialize an empty graph */
    Graph (const ompl::base::SpaceInformationPtr &si)
    : boost_graph(), si_(si)
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
    
    /** \brief Compute the length of a path by summing the distance between states
    * 
    * @param path      path to find the length of\
    * 
    * @return          length of path
    */
    double pathLength (const std::list<Vertex> &path) const;

    /** \brief Find the shortest path between two vertices, honoring vertex avoidance
     * 
     * @tparam N        Neighborhood class to use
     * 
     * @param start                 starting vertex
     * @param end                   ending vertex
     * @param avoidNeighborhoods    vector of objects of type N to avoid
     * 
     * @return          list of the vertices in the path from start to end; empty list if no path was found
     */
    template <class N>
    Path getShortestPathWithAvoidance (Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods) const;
};

struct StateSpaceNeighborhood
{
    typedef ompl::base::State *center_type;
    typedef double radius_type;
    
    const Graph &g;
    center_type center;
    radius_type radius;
    
    StateSpaceNeighborhood (const Graph &g, const center_type c, const radius_type r)
    : g(g), center(c), radius(r)
    {
    }
    
    StateSpaceNeighborhood &operator=(const StateSpaceNeighborhood &copy)
    {
        StateSpaceNeighborhood *nbh = new StateSpaceNeighborhood(copy.g, copy.center, copy.radius);
        return *nbh;
    }
    
    bool shouldAvoid (const Edge e) const
    {
        const Vertex u = boost::source(e, g);
        const Vertex v = boost::target(e, g);
        return isInside(u) || isInside(v);
    }
    
private:
    
    bool isInside (const Vertex u) const
    {
        return g.getSpaceInformation()->distance(boost::get(boost::vertex_prop, g, u).state, center) <= radius;
    }
};

struct GraphDistanceNeighborhood
{
    typedef Vertex center_type;
    typedef double radius_type;
    
    const Graph &g;
    center_type center;
    radius_type radius;
    
    GraphDistanceNeighborhood (const Graph &g, const center_type c, const radius_type r)
    : g(g), center(c), radius(r)
    {
    }
    
    GraphDistanceNeighborhood &operator=(const GraphDistanceNeighborhood &copy)
    {
        GraphDistanceNeighborhood *nbh = new GraphDistanceNeighborhood(copy.g, copy.center, copy.radius);
        return *nbh;
    }
    
    bool shouldAvoid (const Edge e) const
    {
        Vertex u = boost::source(e, g);
        Vertex v = boost::target(e, g);
        return isInside(u, radius) || isInside(v, radius);
    }
    
private:
    
    // !!! This is really slow
    bool isInside (const Vertex u, const radius_type r) const
    {
        if (r < 0)
            return false;
        if (u == center)
            return true;
        
        BOOST_FOREACH(const Vertex k, boost::adjacent_vertices(u, g))
        {
            double edgeWeight = boost::get(boost::edge_weight, g, boost::edge(u, k, g).first);
            if (isInside(k, r - edgeWeight))
                return true;
        }
        return false;
    }
};

struct SingleEdgeNeighborhood
{
    Edge edge;
    
    SingleEdgeNeighborhood (const Edge e)
    : edge(e)
    {
    }
    
    bool shouldAvoid (const Edge e) const
    {
        return e == edge;
    }
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
    
public:
    
    typedef Edge key_type;
    typedef double value_type;
    typedef double reference;
    typedef boost::readable_property_map_tag category;
    
    edgeWeightMap (const Graph &graph, Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods)
    : g(graph), start(start), end(end), avoid(avoidNeighborhoods)
    {
    }
    
    bool shouldAvoid (Edge e) const;
    
    const boost_graph &getGraph (void) const
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

struct foundGoalException
{
};

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

#endif
