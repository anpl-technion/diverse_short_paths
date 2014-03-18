/*
 * DStar.h
 */

#ifndef __D_STAR_H
#define __D_STAR_H

#include "pch.h"

#include "Graph.h"
#include "Neighborhood.h"
#include "Path.h"

/*
 * Koenig and Likhachev's D* lite algorithm, adapted from an implementation by
 * Arnaud Degroote <https://github.com/adegroote/gladys/blob/master/include/gladys/dstar.hpp>
 */
class DStar
{
private:

    typedef std::pair<double, double> Key;
    typedef boost::bimaps::bimap <
        boost::bimaps::unordered_set_of<Vertex>,
        boost::bimaps::multiset_of<Key, std::less<Key>>
    > Bimap;
    typedef Bimap::value_type BimapValue;
    struct cost {
        double g;
        double rhs;
    };

    const Graph &g;
    Vertex start, goal, last;
    Bimap pq;
    std::vector<cost> costs;
    double keyModifier;
    
    std::set<Edge> avoiding;
    std::set<Edge> updatedEdges;

public:
    // Generate a fresh DStar search from a graph
    DStar (const Graph &g, const Vertex start, const Vertex goal);
    
    // Generate a new DStar search from an old one under additional constraint
    DStar (const DStar *old, const Neighborhood &avoid);
    
    const Graph &getGraph () const;
    Vertex getStart () const;
    Vertex getGoal () const;
    Bimap getPQ () const;
    std::vector<cost> getCosts () const;
    double getKeyModifier () const;
    std::set<Edge> getAvoiding() const;
    
    // Extract the path from the algorithm's results
    Path getPath () const;
    
private:
    
    double edgeWeight (const Edge e) const;
    
    Key calculateKey (const Vertex v) const;

    double h (const Vertex u, const Vertex v) const;

    void updateVertex(const Vertex v);
    
    void computeShortestPath();
    
    void replan (const Vertex v);
    
};

#endif

