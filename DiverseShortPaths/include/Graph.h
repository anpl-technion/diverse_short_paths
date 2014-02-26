/*
 * Graph.h
 */

#ifndef __GRAPH_H
#define __GRAPH_H

#include "pch.h"

#include "_graph_detail.h"

class Path;

/*
 * Wrapper for a boost graph, with helper functions
 */
class Graph : public boost_graph
{
private:
    ompl::base::SpaceInformationPtr si;
    
public:
    // Initialize an empty graph
    Graph (const ompl::base::SpaceInformationPtr &sinfo);
    
    // Initialize graph from GraphML file
    Graph (const ompl::base::SpaceInformationPtr &sinfo, std::istream &graphml);
    
    // Destroy graph
    ~Graph ();
    
    // Get the SpaceInformationPtr
    ompl::base::SpaceInformationPtr getSpaceInfo () const;
    
    // Add a vertex to this graph with a specified state
    Vertex addVertex (ompl::base::State *state);
    
    // Add an edge to this graph between two vertices, with an edge weight
    Edge addEdge (const Vertex &u, const Vertex &v);
    
    // Get the number of vertices in this graph
    std::size_t getNumVertices () const;
    
    // Get the length of an edge in the state space
    double getEdgeWeight (const Edge e) const;
    
    // Get the length of an edge, specified by its vertices
    double getEdgeWeight (const Vertex u, const Vertex v) const;
    
    // Get the ompl state for this vertex
    ompl::base::State *getVertexState (const Vertex v) const;
    
    // Get an edge's two vertices
    void getVertices (const Edge e, Vertex *const u, Vertex *const v) const;
    
    // Apply a function to every edge in this graph
    void foreachEdge (std::function<void (const Edge)> applyMe) const;
    
    // Apply a function to every vertex in this graph
    void foreachVertex (std::function<void (const Vertex)> applyMe) const;
    
    // Compute the Levenshtein edit distance between two paths
    double levenshteinDistance (const Path &path1, const Path &path2) const;
    
    // Compute the midpoint state of two states
    void midpoint (const ompl::base::State *s1, const ompl::base::State *s2, ompl::base::State *mid) const;
};

#endif
