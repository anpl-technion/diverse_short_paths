/*
 * Graph.h
 */

#ifndef __GRAPH_H
#define __GRAPH_H

#include "pch.h"

#include "_graph_detail.h"

/**
 * Wrapper for a boost graph with convenient functions.
 */
class Graph : public boost_graph
{
private:
    
    const ompl::base::SpaceInformationPtr si;   // Information about the space the graph is embedded in
    mutable double **apsp;      // May hold precomputed pairwise distances between all vertices
    
public:
    
    /**
     * Construct graph from GraphML file.
     * @param si        our space's information
     * @param graphml   input in graphml format
     */
    Graph (const ompl::base::SpaceInformationPtr &si, std::istream &graphml);
    
    /** Destructor. */
    ~Graph ();
    
    /**
     * Get the SpaceInformationPtr.
     * @return our \a si
     */
    const ompl::base::SpaceInformationPtr getSpaceInfo () const;
    
    /**
     * Get the number of vertices.
     * @return our vertex count
     */
    std::size_t getNumVertices () const;
    
    /**
     * Get the edge connecting two vertices
     * @param u first vertex in graph
     * @param v second vertex in graph
     * @return edge that connects \a u to \a v
     * @warning Return value undefined if no such edge exists.
     */
    Edge getEdge (Vertex u, Vertex v) const;
    
    /**
     * Get the length of an edge in C-space.
     * @param e edge in graph
     * @return length (i.e. weight) of \a e
     */
    double getEdgeWeight (Edge e) const;
    
    /**
     * Get the length of an edge, specified by its vertices.
     * @param u first vertex in graph
     * @param v second vertex in graph
     * @return length (i.e. weight) of the edge that connects \a u to \a v
     * @warning Return value undefined if no such edge exists (may even crash?).
     */
    double getEdgeWeight (Vertex u, Vertex v) const;
    
    /**
     * Get the ompl state for a vertex.
     * @param v vertex in the graph
     * @return state in C-space of \a v
     */
    const ompl::base::State *getVertexState (Vertex v) const;
    
    /**
     * Get an edge's two vertices.
     * @param e         edge in graph\
     * @return tuple of source and destination vertices of \a e
     */
    std::tuple<Vertex,Vertex> getVertices (Edge e) const;
    
    /**
     * Apply a function to every edge in this graph.
     * @param applyMe   function operating on an edge
     */
    void foreachEdge (std::function<void (Edge)> applyMe) const;
    
    /**
     * Apply a function to every vertex in this graph.
     * @param applyMe   function operating on a vertex
     */
    void foreachVertex (std::function<void (Vertex)> applyMe) const;
    
    /**
     * Compute the midpoint of two states.
     * @param s1        first state in C-space
     * @param s2        second state in C-space
     * @param mid[out]  allocated state in C-space
     * @return midpoint of \a s1, \a s2 in \a mid
     */
    void midpoint (const ompl::base::State *s1, const ompl::base::State *s2, ompl::base::State *mid) const;
    
    /** Pre-compute the graph distance between every pair of vertices. */
    void allPairsShortestPaths () const;
    
    /**
     * Compute the distance along graph edges between two vertices.
     * @param u first vertex in graph
     * @param v second vertex in graph
     * @return graph distance between \a u and \a v
     */
    double graphDistance (Vertex u, Vertex v) const;
};

#endif
