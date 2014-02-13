/*
 * Path.h
 */

#ifndef __PATH_H
#define __PATH_H

#include "pch.h"

#include "_graph_detail.h"

class Path : public std::vector<Vertex>
{
private:
    Graph *g;
    mutable double length;
    mutable bool dirty;
    
public:
    
    Path (const Path &path);
    
    Path (std::vector<Vertex> &path, Graph *g);
    
    Path (Graph *g);
    
    double getLength () const;
    
    Graph *getGraph () const;
    
    void print () const;
    
    void clear ();
    
    void push_back (const Vertex &vertex);
    
private:
    
    void computeLength () const;
};


#endif
