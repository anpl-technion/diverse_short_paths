/*
 * Neighborhood.h
 */

#ifndef __NEIGHBORHOOD_H
#define __NEIGHBORHOOD_H

#include "pch.h"

#include "_graph_detail.h"

/**
 * A region in a graph defined by a center and a radius.
 */
class Neighborhood
{
public:
    
    /** Ways distance can be measured from the center. */
    enum AvoidMethod {
        UNKNOWN,    ///< Initial value for a\ method; must be changed
        CSPACE,     ///< Use C-space distance metric
        GRAPH       ///< Use graph distance metric
    };
    
private:
    
    static const Graph *graph;                  // Graph regions lie in
    static ompl::base::State **statePool;       // Set of pre-allocated states to work with
    static AvoidMethod method;                  // Setting for distance measure
    
    ompl::base::State *center;                  // State at the center of region
    Edge centerEdge;                            // Edge center lies on
    double edgeWeight;                          // C-space length of the edge
    Vertex centerU;                             // Source vertex of the edge
    Vertex centerV;                             // Destination vertex of the edge
    double centerWeight;                        // C-space distance between center and state of centerU
    double radius;                              // Radius of the region
    
public:
    
    /**
     * Construct a new region.
     * @param c     state at region's center
     * @param cedge edge that \a c lies on
     * @param r     radius of region
     * @warning Neighborhood will free \a c upon destruction.
     */
    Neighborhood (ompl::base::State *c, Edge cedge, double r);
    
    /**
     * Copy constructor.
     * @param copy  neighborhood to deep duplicate
     */
    Neighborhood (const Neighborhood &copy);
    
    /**
     * Copy assignment.
     * @param copy  neighborhood to deep duplicate
     */
    Neighborhood &operator=(const Neighborhood &copy);
    
    /** Destructor. */
    ~Neighborhood ();
    
    /**
     * Initializations for all neighborhoods.
     * @param m     method to use to measure distance to our \a center
     * @param graph graph region will lie in
     * @note Must be called prior to using neighborhoods.
     * @note Call destroySharedResources() when finished with neighborhoods.
     */
    static void constructSharedResources (AvoidMethod m, const Graph *graph);
    
    /**
     * Destroy resources shared by all regions.
     * @warning After calling this, constructSharedResources() must be called again
     *  before neighborhoods will work again.
     */
    static void destroySharedResources ();
    
    /**
     * Get center state of the region.
     * @return our \a center
     */
    const ompl::base::State *getCenter () const;
    
    /**
     * Get edge center lies on.
     * @return our \a centerEdge
     * */
    Edge getCenterEdge () const;
    
    /**
     * Get radius of the region.
     * @return our \a radius
     */
    double getRadius () const;
    
    /**
     * Does edge intersect this region?
     * @param e edge in the \a graph
     * @return true if any point along \a e is closer to our \a center than our \a radius
     *  according to the distance measure selected by the \a method; false otherwise
     */
    bool shouldAvoid (Edge e) const;
    
private:
    
    /** Initialize internal values for \a AvoidMethod::GRAPH mode. */
    void setupWeight ();
    
    /**
     * Does edge intersect this region in C-space distance?
     * @param e edge in the \a graph
     * @return true if any point along \a e is closer to our \a center than
     *  out \a radius according to C-space distance; false otherwise
     */
    bool shouldAvoid_cspace (Edge e) const;
    
    /**
     * Does edge intersect this region in graph distance?
     * @param e edge in the \a graph
     * @return true if any point along \a e is closer to our \a center than
     *  out \a radius according to graph distance; false otherwise
     */
    bool shouldAvoid_graph (Edge e) const;
    
    /**
     * Is state within this region in C-space distance?
     * @param s arbitrary state in C-space
     * @return true if \a s is within this region; false otherwise
     */
    bool isInside (const ompl::base::State *s) const;
    
};

#endif
