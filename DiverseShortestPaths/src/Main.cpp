
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "Graph.h"

// Magic numbers
#define NUM_AVOIDS_PER_PATH     5
#define AVOID_RADIUS_FACTOR     0.1

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
    // Don't allow start or end states to be sampled (waste of time)
    for (std::size_t i = 1 + std::rand() % (path.size()-2); i > 0; i--)
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
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(-10);
    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add 10 random states and connect them randomly
    for (int i = 0; i < 10; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    
    // Make some edges
    g.addEdge(boost::vertex(0, g), boost::vertex(1, g));
    g.addEdge(boost::vertex(0, g), boost::vertex(3, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(4, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(5, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(6, g));
    g.addEdge(boost::vertex(5, g), boost::vertex(6, g));
    g.addEdge(boost::vertex(6, g), boost::vertex(2, g));
    g.addEdge(boost::vertex(6, g), boost::vertex(9, g));
    g.addEdge(boost::vertex(2, g), boost::vertex(7, g));
    g.addEdge(boost::vertex(7, g), boost::vertex(8, g));
    g.addEdge(boost::vertex(9, g), boost::vertex(8, g));
    
    std::vector<std::list<Vertex> > paths = findDiverseShortestPaths(4, boost::vertex(0, g), boost::vertex(8, g), g);
    BOOST_FOREACH(std::list<Vertex> path, paths)
    {
        std::cout << "Path:\n";
        BOOST_FOREACH(Vertex v, path)
        {
            std::cout << v << "\n";
        }
        std::cout << "\n\n";
    }
    
    return 0;
}

