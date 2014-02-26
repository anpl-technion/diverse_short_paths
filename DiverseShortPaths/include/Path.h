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
    std::vector<double> parametrization;
    
public:
    
    Path (const Path &path);
    
    Path (std::vector<Vertex> &path, Graph *g);
    
    Path (Graph *g);
    
    double getLength () const;
    
    Graph *getGraph () const;
    
    void print () const;
    
    void printWithWeights () const;
    
    void push_back (const Vertex &vertex);
    
    std::vector<double> getPartialEdgeSums () const;
    
    // Allocates a state that should be freed later
    ompl::base::State *sampleUniform () const;
    
    static double distance (const Path &p1, const Path &p2);
};


#endif
