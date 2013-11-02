
/* Author: Caleb Voss */

#include "Path.h"
#include "Graph.h"

Path::Path (std::list<Vertex> &path, Graph &g)
: _path(path), _delete(false)
{
    _length = g.pathLength(path);
}

Path::Path (std::list<Vertex> &path, const Graph &g)
: _path(path), _delete(false)
{
    _length = g.pathLength(path);
}
