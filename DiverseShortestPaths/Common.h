
/* Author: Caleb Voss */

#ifndef COMMON_H
#define COMMON_H

#include <boost/graph/adjacency_list.hpp>

#define NBH_STATESPACE false
#define NBH_GRAPHDISTANCE true

class Graph;
struct VertexAttributes;

// Will allow us to associate an object to each vertex representing all its attributes
namespace boost {
    enum vertex_prop_t { vertex_prop };
}

// Wrapper for the object assigned to a vertex as its property
typedef boost::property <boost::vertex_prop_t,VertexAttributes> VertexProperty;
// Wrapper for the object assigned to an edge as its property
typedef boost::property <boost::edge_weight_t,double> EdgeProperty;
// The underlying boost graph type (undirected weighted-edge adjacency list with embedding)
typedef boost::adjacency_list <
    boost::vecS, boost::vecS, boost::undirectedS,
    VertexProperty,
    EdgeProperty> boost_graph;
// Our graph's graph_traits
typedef boost::graph_traits<boost_graph> graph_traits;
// Vertex type
typedef graph_traits::vertex_descriptor Vertex;
// Edge type
typedef graph_traits::edge_descriptor Edge;

struct StateSpaceNeighborhood;
struct GraphDistanceNeighborhood;
struct SingleEdgeNeighborhood;

class Path;
class MutablePath;

#endif
