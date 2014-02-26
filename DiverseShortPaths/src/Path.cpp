/*
 * Path.cpp
 */

#include "Path.h"

#include "Graph.h"

Path::Path (const Path &path)
 :  std::vector<Vertex>(path), g(path.getGraph()), parametrization(path.getPartialEdgeSums())
{ }

Path::Path (std::vector<Vertex> &path, Graph *graph)
 :  std::vector<Vertex>(path), g(graph)
{
    parametrization.reserve(size());
    parametrization.push_back(0);
    for (std::size_t i = 1; i < size(); i++)
    {
        parametrization.push_back(parametrization[i-1] + g->getEdgeWeight((*this)[i-1], (*this)[i]));
    }
}

Path::Path (Graph *graph)
 :  std::vector<Vertex>(), g(graph)
{ }

double Path::getLength () const
{
    return parametrization[size()-1];
}

Graph *Path::getGraph () const
{
    return g;
}

void Path::print () const
{
    for (std::size_t i = 0; i < size(); i++)
    {
        std::cout << (*this)[i] << " ";
    }
    std::cout << ": " << getLength() << "\n";
}

void Path::printWithWeights () const
{
    for (std::size_t i = 0; i < size()-1; i++)
    {
        const double w = g->getEdgeWeight((*this)[i], (*this)[i+1]);
        std::cout << (*this)[i] << " (" << w << ") ";
    }
    std::cout << (*this)[size()-1] << " : " << getLength() << "\n";
}

void Path::push_back (const Vertex &vertex)
{
    std::vector<Vertex>::push_back(vertex);
    if (size() == 1)
        parametrization.push_back(0);
    else
        parametrization.push_back(parametrization[size()-2]
            + g->getEdgeWeight((*this)[size()-2], (*this)[size()-1]));
}

std::vector<double> Path::getPartialEdgeSums () const
{
    return parametrization;
}

ompl::base::State *Path::sampleUniform () const
{
    // Sample between [0,length]
    double par = getLength() * ((double)rand() / (double)RAND_MAX);
    
    // Find vertices to interpolate between
    std::size_t i = 0;
    while (!(parametrization[i] <= par && par <= parametrization[i+1]))
    {
        i++;
    }
    
    par = (par-parametrization[i]) / getLength();
    
    // Interpolate
    ompl::base::State *sample = g->getSpaceInfo()->allocState();
    g->getSpaceInfo()->getStateSpace()->interpolate(g->getVertexState((*this)[i]), g->getVertexState((*this)[i+1]), par, sample);
    return sample;
}

double Path::distance (const Path &p1, const Path &p2)
{
    if (p1.getGraph() != p2.getGraph())
        return std::numeric_limits<double>::quiet_NaN();
    return p1.getGraph()->levenshteinDistance(p1, p2);
}