
#ifndef PATH_H
#define PATH_H

#include <list>

#include "Graph.h"

class Path
{
    std::list<Vertex> _path;
    double _length;
    mutable bool _delete;
    
public:
    
    Path (const Path &path)
    : _path(path.getPath()), _length(path.getLength()), _delete(false)
    {
    }
    
    Path (std::list<Vertex> &path, Graph &g)
    : _path(path), _delete(false)
    {
        _length = pathLength(path, g);
    }
    
    Path (std::list<Vertex> &path, const Graph &g)
    : _path(path), _delete(false)
    {
        _length = pathLength(path, g);
    }
    
    void markForDeletion (void) const
    {
        _delete = true;
    }
    
    bool isMarkedForDeletion (void) const
    {
        return _delete;
    }
    
    double getLength (void) const
    {
        return _length;
    }
    
    std::list<Vertex> &getPath (void)
    {
        return _path;
    }
    
    const std::list<Vertex> &getPath (void) const
    {
        return _path;
    }
    
    typedef std::list<Vertex>::iterator iterator;
    
    iterator begin (void)
    {
        return _path.begin();
    }
    
    iterator end (void)
    {
        return _path.end();
    }
};

unsigned int levenshtein (const std::list<Vertex> &path1, const std::list<Vertex> &path2);

#endif
