/*
 * _graph_detail.h
 */

#ifndef __GRAPH_DETAIL_H
#define __GRAPH_DETAIL_H

#include "pch.h"

class Graph;
struct Neighborhood;

// Will allow us to associate an object to each vertex representing all its attributes
namespace boost
{
    enum vertex_prop_t { vertex_prop };
}

// Collection of attributes for a vertex
struct VertexAttributes
{
    // Vertex's location in the state space in which the graph is embedded
    ompl::base::State *state;
    
    VertexAttributes ();
    
    VertexAttributes (ompl::base::State *state);
};

// Wrapper for the object assigned to a vertex as its property
typedef boost::property <boost::vertex_prop_t,VertexAttributes> VertexProperty;
// Wrapper for the object assigned to an edge as its property
typedef boost::property <boost::edge_weight_t,double> EdgeProperty;
// The underlying boost graph type (undirected weighted-edge adjacency list with embedding)
typedef boost::adjacency_list <
  boost::vecS, boost::vecS, boost::directedS,
  VertexProperty, EdgeProperty> boost_graph;
// Our graph's graph_traits
typedef boost::graph_traits<boost_graph> graph_traits;
// Vertex type
typedef graph_traits::vertex_descriptor Vertex;
// Edge type
typedef graph_traits::edge_descriptor Edge;

namespace boost
{
    BOOST_INSTALL_PROPERTY(vertex, prop);
}

class heuristic // implements AStarHeuristic
{
    const Graph &g;
    const Vertex goal;
    
public:
    
    heuristic (const Graph &graph, Vertex goal);
    
    double operator() (Vertex u) const;
};

class edgeWeightMap // implements ReadablePropertyMap
{
    const Graph &g;
    const std::vector<Neighborhood> &avoid;
    
public:
    
    typedef Edge key_type;
    typedef double value_type;
    typedef double reference;
    typedef boost::readable_property_map_tag category;
    
    edgeWeightMap (const Graph &graph, const std::vector<Neighborhood> &avoidThese);
    
    bool shouldAvoid (Edge e) const;
    
    const Graph &getGraph () const;
};

namespace boost
{
    double get (const edgeWeightMap &m, const Edge &e);
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
