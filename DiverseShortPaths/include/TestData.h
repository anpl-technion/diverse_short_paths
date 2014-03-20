/*
 * TestData.h
 */

#ifndef __TEST_DATA_H
#define __TEST_DATA_H

#include "pch.h"

#include "_graph_detail.h"

/*
 * Suite of graphs and parameters to test an algorithm.
 */
class TestData
{
private:
    Vertex start;
    Vertex end;
    std::size_t k;
    double maxLength;
    double minDistance;
    const Graph *graph;
    const std::string graphName;
    mutable char *buf;
    
public:
    // Constructor
    TestData (const std::string &graphFileName, const std::size_t numPaths,
              const double maxPathLength, const double minPathPairwiseDistance);
    // Destructor
    ~TestData ();
    
    // Get the graph
    const Graph &getGraph () const;
    
    // Get the basename of the graphml file
    char *getName () const;
    
    // Get the start node
    Vertex getStart () const;
    
    // Get the end node
    Vertex getEnd () const;
    
    // Get the maximum allowable path length
    double getMaxLength () const;
    
    // Get the minimum allowable path closeness
    double getMinDistance () const;
    
    // Get the number of paths to find
    std::size_t getK () const;
    
private:
    // Constructor an empty data set
    TestData ();
};

#endif
