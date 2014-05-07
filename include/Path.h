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
    
    const Graph *g;
    
    std::vector<double> parametrization;
    
    mutable std::vector<double> weights;
    
    mutable std::vector<const ompl::base::State *> states;
    
    bool isWeightCached () const;

    void cacheWeights () const;
    
    bool isStateCached () const;

    void cacheStates () const;
    
public:
    
    Path ();
    
    Path (const Path &path);
    
    Path (std::vector<Vertex> &path, const Graph *g);
    
    Path (const Graph *g);
    
    void saveOMPLFormat(std::ostream &out) const;
    
    double getLength () const;
    
    const Graph *getGraph () const;
    
    void print () const;
    
    void printWithWeights () const;
    
    void push_back (const Vertex &vertex);
    
    std::vector<double> getPartialEdgeSums () const;
    
    // Allocates a state that should be freed later
    ompl::base::State *sampleUniform (Edge *edge) const;
    
    const std::vector<double> &getWeights () const;
    
    const std::vector<const ompl::base::State *> &getStates () const;
    
    static double distance (const Path &p1, const Path &p2);
};


#endif
