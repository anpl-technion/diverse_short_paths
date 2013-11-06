
/* Author: Caleb Voss */

#include "Path.h"

#include "Graph.h"

Path::Path (const Path &path)
: std::list<Vertex>(path), length(path.getLength())
{
}

Path::Path (std::list<Vertex> &path, Graph &g)
: std::list<Vertex>(path)
{
    length = g.pathLength(*this);
}

Path::Path (std::list<Vertex> &path, const Graph &g)
: std::list<Vertex>(path)
{
    length = g.pathLength(*this);
}

bool Path::operator==(Path &rhs)
{
    if (size() != rhs.size())
        return false;
    
    const_iterator his = rhs.begin();
    BOOST_FOREACH(Vertex u, *this)
    {
        if (u != *his)
            return false;
        his++;
    }
    return true;
}

bool Path::sampleable (void) const
{
    return size() != sampled.size();
}

Vertex Path::sampleVertex (void) const
{
    Path::const_iterator vi = begin();
    for (std::size_t i = std::rand() % (size()-sampled.size()); i > 0; i--)
    {
        // Don't allow sampling of vertices already sampled
        if (sampled.find(*vi) != sampled.end())
            i++;
        vi++;
    }
    
    sampled.insert(*vi);
    return *vi;
}

ompl::base::State *Path::sampleState (const Graph &g) const
{
    return boost::get(boost::vertex_prop, g, sampleVertex()).state;
}
    
double Path::getLength (void) const
{
    return length;
}

unsigned int Path::levenshtein (const Path &path1, const Path &path2)
{
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


MutablePath::MutablePath (Path &path)
: path(path), freed(false)
{
}

void MutablePath::free (void) const
{
    path.clear();
    freed = true;
}

bool MutablePath::isFreed (void) const
{
    return freed;
}
