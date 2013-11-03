
/* Author: Caleb Voss */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>

#include <ompl/base/SpaceInformation.h>

#include "Common.h"

namespace boost {
    BOOST_INSTALL_PROPERTY(vertex, prop);
}

// Collection of attributes for a vertex
struct VertexAttributes
{
    // Vertex's location in the state space in which the graph is embedded
    ompl::base::State *state;
    
    VertexAttributes ();
    
    VertexAttributes (ompl::base::State *state);
};

class Graph : public boost_graph
{
private:
    
    ompl::base::SpaceInformationPtr si_;
    
public:
    
    /** \brief Initialize an empty graph */
    Graph (const ompl::base::SpaceInformationPtr &si);
    
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
    double pathLength (const Path &path) const;

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

class heuristic // implements AStarHeuristic
{
    const Graph &g;
    const Vertex goal;
    
public:
    
    heuristic (const Graph &graph, Vertex goal);
    
    double operator() (Vertex u) const;
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
    
    edgeWeightMap (const Graph &graph, Vertex start, Vertex end, const std::vector<N> &avoidNeighborhoods);
    
    bool shouldAvoid (Edge e) const;
    
    const Graph &getGraph (void) const;
};

namespace boost
{
    template <typename K, typename N>
    double get (const edgeWeightMap<N> &m, const K &e);
}

struct foundGoalException
{
};

class visitor : public boost::default_astar_visitor // implements AStar_Visitor
{
    Vertex goal;
    
public:
    
    visitor (Vertex goal);
    
    void examine_vertex(Vertex u, const Graph &g) const;
};

#endif
