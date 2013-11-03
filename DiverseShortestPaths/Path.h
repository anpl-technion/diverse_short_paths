
/* Author: Caleb Voss */

#ifndef PATH_H
#define PATH_H

#include <list>

#include <boost/foreach.hpp>

#include "Common.h"

class Path : public std::list<Vertex>
{
    double length;
    
public:
    
    Path (const Path &path);
    
    Path (std::list<Vertex> &path, Graph &g);
    
    Path (std::list<Vertex> &path, const Graph &g);
    
    bool operator==(Path &rhs);
    
    double getLength (void) const;

    static unsigned int levenshtein (const Path &path1, const Path &path2);

};

struct MutablePath
{
    mutable Path path;
    mutable bool freed;
    
    MutablePath (Path &path);
    
    void free (void) const;
    
    bool isFreed (void) const;
};

#endif
