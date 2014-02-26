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
public:
    enum Mode {UNSET, FIX_MAX_PATH_LENGTH, FIX_MIN_PATH_DISTANCE};
    
private:
    Vertex start;
    Vertex end;
    std::size_t k;
    double maxLength;
    double minDistance;
    Mode mode;
    const Graph *graph;
    
public:
    // Constructor
    TestData (const char *graphFileName, const std::size_t numPaths,
              const double maxPathLength, const double minPathPairwiseDistance);
    // Destructor
    ~TestData ();
    
    // Get the graph
    const Graph &getGraph () const;
    
    // Set the test mode
    void setMode (Mode m);
    
    // Get the start node
    Vertex getStart () const;
    
    // Get the end node
    Vertex getEnd () const;
    
    // Get the maximum allowable path length
    double getMaxLength () const;
    
    // Get the minimum allowable path closeness
    double getMinDistance () const;
    
    std::string getParameter () const;
    
    // Get the number of paths to find
    double getK () const;
    
private:
    // Constructor an empty data set
    TestData ();
};

#endif
