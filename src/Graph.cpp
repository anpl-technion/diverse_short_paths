/*
 * Graph.cpp
 */

#include "Graph.h"

#include "Path.h"
#include "Neighborhood.h"

Graph::Graph (const ompl::base::SpaceInformationPtr &sinfo, std::istream &graphml)
: boost_graph(), si(sinfo), apsp(NULL)
{
    std::map<Vertex, std::string> coordStrings;
    std::map<Edge, double> weights;
    boost::associative_property_map<std::map<Vertex, std::string> > coords_map(coordStrings);
    boost::associative_property_map<std::map<Edge, double> > weight_map(weights);
    boost::dynamic_properties dyn_prop;
    dyn_prop.property("coords", coords_map);
    dyn_prop.property("weight", weight_map);
    boost::read_graphml(graphml, *this, dyn_prop);
    
    foreachEdge([&] (const Edge e) -> void
    {
        boost::put(boost::edge_weight, *this, e, weight_map[e]);
    });
    
    // Populate state information
    foreachVertex([&] (const Vertex v) -> void
    {
        ompl::base::State *state = si->allocState();
        std::istringstream coords(coordStrings[v]);
        std::string coord;
        std::vector<double> reals;
        while (!coords.eof())
        {
            std::getline(coords, coord, ',');
            reals.push_back(std::atof(coord.c_str()));
        }
        si->getStateSpace()->copyFromReals(state, reals);
        si->getStateSpace()->enforceBounds(state);
        boost::put(boost::vertex_prop, *this, v, state);
    });
}

Graph::~Graph ()
{
    foreachVertex([&] (const Vertex v) -> void
    {
        si->freeState(boost::get(boost::vertex_prop, *this, v).state);
    });
}

ompl::base::SpaceInformationPtr Graph::getSpaceInfo () const
{
    return si;
}

Vertex Graph::addVertex (ompl::base::State *state)
{
    return boost::add_vertex(VertexProperty(VertexAttributes(state)), *this);
}

Edge Graph::addEdge (const Vertex &u, const Vertex &v)
{
    ompl::base::State *uState = boost::get(boost::vertex_prop, *this, u).state;
    ompl::base::State *vState = boost::get(boost::vertex_prop, *this, v).state;
    return boost::add_edge(u, v, EdgeProperty(si->distance(uState, vState)), *this).first;
}

std::size_t Graph::getNumVertices () const
{
    return boost::num_vertices(*this);
}

Edge Graph::getEdge (const Vertex u, const Vertex v) const
{
    return boost::edge(u, v, *this).first;
}

double Graph::getEdgeWeight (const Edge e) const
{
    return boost::get(boost::edge_weight, *this, e);
}

double Graph::getEdgeWeight (const Vertex u, const Vertex v) const
{
    return getEdgeWeight(getEdge(u, v));
}

ompl::base::State *Graph::getVertexState (const Vertex v) const
{
    return boost::get(boost::vertex_prop, *this, v).state;
}

void Graph::getVertices (const Edge e, Vertex *const u, Vertex *const v) const
{
    if (u != NULL)
        *u = boost::source(e, *this);
    if (v != NULL)
        *v = boost::target(e, *this);
}

void Graph::foreachEdge (std::function<void (const Edge)> applyMe) const
{
    BOOST_FOREACH (const Edge e, boost::edges(*this))
    {
        applyMe(e);
    }
}

void Graph::foreachVertex (std::function<void (const Vertex)> applyMe) const
{
    BOOST_FOREACH (const Vertex v, boost::vertices(*this))
    {
        applyMe(v);
    }
}

void Graph::midpoint (const ompl::base::State *s1, const ompl::base::State *s2, ompl::base::State *mid) const
{
    si->getStateSpace()->interpolate(s1, s2, 0.5, mid);
}

void Graph::allPairsShortestPaths () const
{
    if (apsp == NULL)
    {
        const std::size_t n = getNumVertices();
        apsp = new double*[n];
        for (std::size_t i = 0; i < n; i++)
            apsp[i] = new double[n];
    }
    
    // Run the Floyd-Warshall algorithm
    boost::floyd_warshall_all_pairs_shortest_paths(*this, apsp);
}

double Graph::graphDistance (Vertex u, Vertex v) const
{
    if (apsp == NULL)
    {
        std::cerr << "Error: All pairs shortest paths not pre-computed!\n";
        exit(-1);
    }
    
    return apsp[u][v];
}
