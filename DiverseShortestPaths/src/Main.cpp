
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cairo/cairo.h>

#include "Graph.h"

// Magic numbers
#define NUM_AVOIDS_PER_PATH     5
#define AVOID_RADIUS_FACTOR     0.05

/** \brief Get a random state from the path (not equal to the start or end of the path)
 * 
 * @param path      path to sample from
 * @param g         graph this path exists in
 * 
 * @return          pointer to a state on the path
 */
ompl::base::State *sampleState (const std::list<Vertex> &path, const Graph &g)
{
    std::list<Vertex>::const_iterator vi = path.begin();
    for (std::size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return boost::get(boost::vertex_prop, g, *vi).state;
}

/** \brief Compute the length of a path by summing the distance between states
 * 
 * @param path      path to find the length of
 * @param g         graph this path exists in
 * 
 * @return          length of path
 */
double pathLength (const std::list<Vertex> &path, const Graph &g)
{
    double length = 0;
    std::list<Vertex>::const_iterator vi = path.begin();
    vi++;
    while (vi != path.end())
    {
        vi--;
        Vertex u = *vi;
        vi++;
        Vertex v = *vi;
        vi++;
        length += boost::get(boost::edge_weight, g, boost::edge(u, v, g).first);
    }
    return length;
}

/** \brief Find a number of diverse, short paths in a graph, by deviating from former short paths through
 *         the use of an increasing set of neighborhoods in the state space that should be avoided
 * 
 * @param numPaths          number of paths to try to find
 * @param start             start vertex for the path
 * @param end               end vertex for the path
 * @param g                 graph to search in
 * 
 * @return                  set of at most numPaths distinct paths from start to end
 */
std::vector<std::list<Vertex> > findDiverseShortestPaths (std::size_t numPaths, Vertex start, Vertex end, const Graph &g)
{
    // Holds the set of paths we've found
    std::vector<std::list<Vertex> > resultPaths;
    if (numPaths == 0)
        return resultPaths;
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<Neighborhood> > resultAvoids;
    // The next path to analyze
    std::size_t frontier = 0;
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<Neighborhood> alreadyAvoiding = std::vector<Neighborhood>();
    std::list<Vertex> referencePath = g.getShortestPathWithAvoidance(start, end, alreadyAvoiding);
    if (referencePath.empty())
        return resultPaths;
    
    resultPaths.push_back(referencePath);
    resultAvoids.push_back(alreadyAvoiding);
    
    // Work through the queue until we have enough
    while (frontier < resultPaths.size() && resultPaths.size() < numPaths)
    {
        referencePath = resultPaths[frontier];
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make a pre-defined number of attempts at imposing a new neighborhood to avoid on the graph
        // Neighborhood's radius is some pre-defined portion of the referencePath's length
        double radius = AVOID_RADIUS_FACTOR * pathLength(referencePath, g);
        for (int i = 0; i < NUM_AVOIDS_PER_PATH && resultPaths.size() < numPaths; i++)
        {
            std::vector<Neighborhood> avoid = alreadyAvoiding;
            avoid.push_back(Neighborhood(sampleState(referencePath, g), radius));
            // Get the shortest path under these constraints
            std::list<Vertex> path = g.getShortestPathWithAvoidance(start, end, avoid);
            if (path.empty())
                continue;
            
            // Don't store it if it's a duplicate
            bool alreadyHave = false;
            BOOST_FOREACH(std::list<Vertex> p, resultPaths)
            {
                if (path == p)
                {
                    alreadyHave = true;
                    break;
                }
            }
            if (!alreadyHave)
            {
                resultPaths.push_back(path);
                resultAvoids.push_back(avoid);
            }
        }
    }
    
    return resultPaths;
}

int main (int argc, char **argv)
{
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add 10 random states and connect them randomly
    for (int i = 0; i < 100; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, g, boost::vertex(i, g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, g, boost::vertex(j, g)).state;
            if (space->distance(iState, jState) < 4)
            {
                if (std::rand() % 4 == 0)
                    g.addEdge(boost::vertex(i, g), boost::vertex(j, g));
            }
        }
    }
    
    std::cout << "Finding at most 10 diverse short paths from node 0 to node 8\n\n";
    std::vector<std::list<Vertex> > paths = findDiverseShortestPaths(10, boost::vertex(0, g), boost::vertex(8, g), g);
    std::cout << "Got " << paths.size() << " paths\n";
    
    cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1024, 1024);
    cairo_t *cr = cairo_create(surface);
    cairo_set_line_width(cr, 1);
    
    BOOST_FOREACH(Vertex v, boost::vertices(g))
    {
        ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
        double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        BOOST_FOREACH(Vertex u, boost::adjacent_vertices(v, g))
        {
            state = boost::get(boost::vertex_prop, g, u).state;
            double *ucoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            
            cairo_move_to(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50);
            cairo_line_to(cr, 12+(ucoords[0]+10)*50, 12+(ucoords[1]+10)*50);
            cairo_stroke(cr);
        }
    }
    
    int line_width = 21;
    float red = 1;
    float green = 0.8;
    float blue = 0;
    BOOST_FOREACH(std::list<Vertex> path, paths)
    {
        cairo_set_line_width(cr, line_width);
        cairo_set_source_rgba(cr, red, green, blue, 1);
        Vertex u = path.front();
        BOOST_FOREACH(Vertex v, path)
        {
            if (u != v)
            {
                ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
                double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                state = boost::get(boost::vertex_prop, g, u).state;
                double *ucoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                
                cairo_move_to(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50);
                cairo_line_to(cr, 12+(ucoords[0]+10)*50, 12+(ucoords[1]+10)*50);
                cairo_stroke(cr);
            }
            u = v;
        }
        line_width -= 2;
        red -= 1.0/11;
        blue += 1.0/11;
    }
    
    BOOST_FOREACH(Vertex v, boost::vertices(g))
    {
        ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
        double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        if (v == boost::vertex(0, g) || v == boost::vertex(8, g))
            cairo_set_source_rgba(cr, 1, 0, 1, 1);
            
        cairo_arc(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50, 8, 0, 2*M_PI);
        cairo_fill(cr);
        cairo_set_source_rgba(cr, 0, 0, 0, 1);
    }
    
    cairo_surface_write_to_png(surface, "output.png");
    return 0;
}

