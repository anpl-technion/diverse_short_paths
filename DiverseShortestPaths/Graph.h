
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

struct Neighborhood
{
    ompl::base::State *center;
    double radius;
    
    Neighborhood (ompl::base::State *c, double r)
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
    ompl::base::SpaceInformationPtr getSpaceInformation (void)
    {
        return si_;
    }
    
    /** \brief Add a vertex to this graph with a specified state
     * 
     * @param state     state associated with this vertex
     * 
     * @return          vertex descriptor of the new vertex
     */
    Vertex addVertex (ompl::base::State *state)
    {
        return boost::add_vertex(VertexProperties(VertexPropCollection(state)), *this);
    }
    
    /** \brief Add an edge to this graph between two vertices, with an edge weight
     *         equal to the StateSpace distance between the vertices' states
     * 
     * @param u     first vertex
     * @param v     second vertex
     * 
     * @return      edge descriptor of the new edge
     */
    Edge addEdge (Vertex u, Vertex v)
    {
        ompl::base::State *uState = boost::get(boost::vertex_prop, *this, u).state;
        ompl::base::State *vState = boost::get(boost::vertex_prop, *this, v).state;
        return boost::add_edge(u, v, EdgeProperties(si_->distance(uState, vState)), *this).first;
    }
        
    /** \brief Find the shortest path between two vertices, honoring vertex avoidance
     * 
     * @param start     starting vertex
     * @param end       ending vertex
     * 
     * @return          list of the vertices in the path from start to end
     */
    std::list<Vertex> getShortestPathWithAvoidance (Vertex start, Vertex end, std::vector<Neighborhood> avoidNeighborhoods);
    
};

class heuristic
{
    Graph &g;
    Vertex goal;
    
public:
    heuristic (Graph &graph, Vertex goal)
    : g(graph), goal(goal)
    {
    }
    
    double operator() (Vertex u)
    {
        ompl::base::State *goalState = boost::get(boost::vertex_prop, g, goal).state;
        ompl::base::State *uState = boost::get(boost::vertex_prop, g, u).state;
        return g.getSpaceInformation()->distance(goalState, uState);
    }
};
    
class edgeWeightMap
{
    Graph &g;
    std::vector<Neighborhood> avoid;
    
public:
    
    edgeWeightMap (Graph &graph, std::vector<Neighborhood> avoidNeighborhoods)
    : g(graph), avoid(avoidNeighborhoods)
    {
    }
    
    bool shouldAvoid (Edge e)
    {
        ompl::base::State *u = boost::get(boost::vertex_prop, g, boost::source(e, g)).state;
        ompl::base::State *v = boost::get(boost::vertex_prop, g, boost::target(e, g)).state;
        BOOST_FOREACH(Neighborhood nbh, avoid)
        {
            if (g.getSpaceInformation()->distance(u, nbh.center) < nbh.radius ||
                g.getSpaceInformation()->distance(v, nbh.center) < nbh.radius)
                return true;
        }
        return false;
    }
    
    base_graph &getGraph (void)
    {
        return g;
    }
};

struct foundGoalException {};

class visitor : public boost::default_astar_visitor
{
    Vertex goal;
    
public:
    visitor (Vertex goal)
    : goal(goal)
    {
    }
    
    void examine_vertex(Vertex u, const Graph &g)
    {
        if (u == goal)
            throw foundGoalException();
    }
};

namespace boost
{
    template <>
    struct property_traits <edgeWeightMap>
    {
        typedef double value_type;
        typedef double reference;
        typedef Edge key_type;
        typedef readable_property_map_tag category;
    };
    
    double get (edgeWeightMap m, Edge e)
    {
        if (m.shouldAvoid(e))
            return std::numeric_limits<double>::max();
        return get(edge_weight, m.getGraph(), e);
    }
}

#endif
