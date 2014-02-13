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
    const Graph *graph;
    Vertex start;
    Vertex end;
    double maxLength;
    double minDistance;
    std::size_t k;
    
public:
    // Destructor
    ~TestData ();
    
    // Generate a set of data from OMPL SPARS problem
    static TestData *generate ();
    
    // Set the graph to run on
    void setGraph (const Graph *g);
    
    // Get the graph
    const Graph &getGraph () const;
    
    // Set the start and end nodes
    void setStartEnd (const Vertex s, const Vertex e);
    
    // Get the start node
    Vertex getStart () const;
    
    // Get the end node
    Vertex getEnd () const;
    
    // Get the maximum allowable path length
    double getMaxLength () const;
    
    // Get the minimum allowable path closeness
    double getMinDistance () const;
    
    // Set the number of paths to find
    void setK (const std::size_t newK);
    
    // Get the number of paths to find
    double getK () const;
    
private:
    // Constructor an empty data set
    TestData ();
};

#endif
