/*
 * Path.cpp
 */

#include "Path.h"

#include "Graph.h"

Path::Path (const Path &path)
 :  std::vector<Vertex>(path), g(path.getGraph()), length(path.getLength()), dirty(false)
{ }

Path::Path (std::vector<Vertex> &path, Graph *graph)
 :  std::vector<Vertex>(path), g(graph)
{
    computeLength();
}

Path::Path (Graph *graph)
 :  std::vector<Vertex>(), g(graph), length(0), dirty(false)
{ }

double Path::getLength () const
{
    if (dirty)
        computeLength();
    return length;
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

void Path::computeLength () const
{
    length = g->computePathLength(*this);
    dirty = false;
}

void Path::clear ()
{
    dirty = true;
    std::vector<Vertex>::clear();
}

void Path::push_back (const Vertex &vertex)
{
    dirty = true;
    std::vector<Vertex>::push_back(vertex);
}

double Path::distance (const Path &p1, const Path &p2)
{
    if (p1.getGraph() != p2.getGraph())
        return std::numeric_limits<double>::quiet_NaN();
    return p1.getGraph()->levenshteinDistance(p1, p2);
}