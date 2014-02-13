/*
 * Graph.cpp
 */

#include "Graph.h"

#include "Path.h"
#include "Neighborhood.h"

Graph::Graph (const ompl::base::SpaceInformationPtr &sinfo)
: boost_graph(), si(sinfo) { }

Graph::Graph (const ompl::base::SpaceInformationPtr &sinfo, std::istream &graphml)
: boost_graph(), si(sinfo)
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

double Graph::getEdgeWeight (const Edge e) const
{
    return boost::get(boost::edge_weight, *this, e);
}

ompl::base::State *Graph::getVertexState (const Vertex v) const
{
    return boost::get(boost::vertex_prop, *this, v).state;
}
    
void Graph::getVertices (const Edge e, Vertex *const u, Vertex *const v) const
{
    *u = boost::source(e, *this);
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

double Graph::computePathLength (const Path &path) const
{
    double length = 0;
    for (std::size_t i = 0; i < path.size() - 1; i++)
    {
        length += getEdgeWeight(boost::edge(path[i], path[i+1], *this).first);
    }
    return length;
}

double Graph::levenshteinDistance (const Path &path1, const Path &path2) const
{
    // TODO: introduce edge weights as a cost
    const std::size_t rowLength = path1.size()+1;
    const std::size_t colLength = path2.size()+1;
    unsigned int *const distances = new unsigned int[rowLength*colLength];
    
    for (std::size_t j = 0; j < rowLength; j++)
        distances[j] = j;
    for (std::size_t i = 1; i < colLength; i++)
        distances[i*rowLength] = i;
    
    Path::const_iterator iIt = path1.begin();
    for (std::size_t i = 1; i < colLength; i++, iIt++)
    {
        Path::const_iterator jIt = path2.begin();
        for (std::size_t j = 1; j < rowLength; j++, jIt++)
        {
            const unsigned int del = distances[(i-1)*rowLength+j] + 1;
            const unsigned int ins = distances[i*rowLength+j-1] + 1;
            unsigned int match = distances[(i-1)*rowLength+j-1];
            if (*iIt != *jIt)
                match++;
            distances[i*rowLength+j] = std::min(std::min(del, ins), match);
        }
    }
        
            const unsigned int d = distances[rowLength*colLength-1];
            delete [] distances;
            return d;
}
