/*
 * Results.h
 */

#ifndef __RESULTS_H
#define __RESULTS_H

#include "pch.h"

#include "_graph_detail.h"

/*
 * Representation of experiment results for an algorithm.
 */
class Results
{
private:
    std::vector<std::vector<Vertex>> paths;
    
public:
    
    // Constructor
    Results ();
    
    // Add path to the set
    void addPath (const std::vector<Vertex> path);
    
    // Combine sets of results into pretty format
    static void collate (const Results *r1, const Results *r2);
};

#endif
