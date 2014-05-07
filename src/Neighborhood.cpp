/*
 * Neighborhood.cpp
 */

#include "Neighborhood.h"

#include "Graph.h"

Neighborhood::Neighborhood (const Graph &g, ompl::base::State **state_pool, ompl::base::State *c, const double r)
 :  g(g), statePool(state_pool), center(c), radius(r)
{
}

Neighborhood::Neighborhood (const Neighborhood &copy)
 :  g(copy.g), statePool(copy.statePool), center(copy.g.getSpaceInfo()->cloneState(copy.center)), radius(copy.radius)
{
}

Neighborhood &Neighborhood::operator=(const Neighborhood &copy)
{
    Neighborhood *nbh = new Neighborhood(copy.g, copy.statePool, copy.g.getSpaceInfo()->cloneState(copy.center), copy.radius);
    return *nbh;
}

Neighborhood::~Neighborhood ()
{
    g.getSpaceInfo()->freeState(center);
}

bool Neighborhood::shouldAvoid (const Edge e) const
{
    Vertex u = 0;
    Vertex v = 0;
    g.getVertices(e, &u, &v);
    
    ompl::base::SpaceInformationPtr si = g.getSpaceInfo();
    
    ompl::base::State *left = statePool[0];
    ompl::base::State *right = statePool[1];
    ompl::base::State *midleft = statePool[2];
    ompl::base::State *midright = statePool[3];
    ompl::base::State *mid = statePool[4];
    
    si->copyState(left, g.getVertexState(u));
    si->copyState(right, g.getVertexState(v));
    g.midpoint(left, right, mid);
    // Might be able to quit early
    if (isInside(left) || isInside(right) || isInside(mid))
        return true;
    
    g.midpoint(left, mid, midleft);
    g.midpoint(right, mid, midright);
    
    // h is always distance(left, mid)
    double h = si->distance(left, mid);
    double left_dist = si->distance(midleft, center);
    double right_dist = si->distance(midright, center);
    
    bool inside = true;
    // Until we find we are inside the neighborhood...
    while ((left_dist = si->distance(midleft, center)) >= radius
        && (right_dist = si->distance(midright, center)) >= radius)
    {
        // Choose the closer interval
        if (left_dist < right_dist)
        {
            ompl::base::State *tmp = right;
            right = mid;
            mid = midleft;
            midleft = tmp;  // So we don't lose track of the 5 allocated states
            
            // If there's no chance of hitting the neighborhood, stop now
            if (left_dist > radius + h)
            {
                inside = false;
                break;
            }
        }
        else
        {
            ompl::base::State *tmp = left;
            left = mid;
            mid = midright;
            midright = tmp;  // So we don't lose track of the 5 allocated states
            
            // If there's no chance of hitting the neighborhood, stop now
            if (right_dist > radius + h/2)
            {
                inside = false;
                break;
            }
        }
        // Make sure h is still distance(left, mid)
        h /= 2;
        
        // Find new interval midpoints
        g.midpoint(left, mid, midleft);
        g.midpoint(right, mid, midright);
    }
    
    return inside;
}

ompl::base::State **Neighborhood::allocStatePool (const Graph &g)
{
    ompl::base::State **statePool = new ompl::base::State *[5];
    for (size_t i = 0; i < 5; i++)
    {
        statePool[i] = g.getSpaceInfo()->allocState();
    }
    return statePool;
}

void Neighborhood::destroyStatePool (const Graph &g, ompl::base::State **statePool)
{
    for (size_t i = 0; i < 5; i++)
    {
        g.getSpaceInfo()->freeState(statePool[i]);
    }
    
    delete [] statePool;
}

bool Neighborhood::isInside (const ompl::base::State *s) const
{
    return g.getSpaceInfo()->distance(s, center) < radius;
}

