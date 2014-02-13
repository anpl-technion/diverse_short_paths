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
