
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "Graph.h"

// Magic numbers
#define NUM_AVOIDS_PER_PATH     5
#define AVOID_RADIUS_FACTOR     0.01

ompl::base::State *sampleState (const std::list<Vertex> &path, const Graph &g)
{
    std::list<Vertex>::const_iterator vi = path.begin();
    for (size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return boost::get(boost::vertex_prop, g, *vi).state;
}

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

std::vector<std::list<Vertex> > &findDiverseShortestPaths (size_t numPaths, Vertex start, Vertex end, const Graph &g,
                                                    const std::list<Vertex> &referencePath = std::list<Vertex>(),
                                                    const std::vector<Neighborhood> &alreadyAvoiding = std::vector<Neighborhood>())
{
    static std::vector<std::list<Vertex> > resultPaths;
    
    if (resultPaths.size() == numPaths)
        return resultPaths;
    
    if (referencePath.empty())
    {
        resultPaths.clear();
        std::list<Vertex> path = g.getShortestPathWithAvoidance(start, end, alreadyAvoiding);
        if (path.empty())
            return resultPaths;
        
        resultPaths.push_back(path);
        std::cout << "Appending path:\n";
        BOOST_FOREACH(Vertex v, path)
        {
            std::cout << v << " " << boost::get(boost::vertex_prop, g, v).state << "\n";
        }
        std::cout << "With avoid nodes: none\n\n";
        return findDiverseShortestPaths(numPaths, start, end, g, path, alreadyAvoiding);
    }
    
    double radius = AVOID_RADIUS_FACTOR * pathLength(referencePath, g);
    for (int i = 0; i < NUM_AVOIDS_PER_PATH && resultPaths.size() < numPaths; i++)
    {
        std::vector<Neighborhood> avoid = alreadyAvoiding;
        avoid.push_back(Neighborhood(sampleState(referencePath, g), radius));
        std::list<Vertex> path = g.getShortestPathWithAvoidance(start, end, avoid);
        if (path.empty())
            continue;
        resultPaths.push_back(path);
        std::cout << "Appending path:\n";
        BOOST_FOREACH(Vertex v, path)
        {
            std::cout << v << " " << boost::get(boost::vertex_prop, g, v).state << "\n";
        }
        std::cout << "With avoid nodes:\n";
        BOOST_FOREACH(Neighborhood nbh, avoid)
        {
            std::cout << nbh.center << "\n";
        }
        std::cout << "\n\n";
        findDiverseShortestPaths(numPaths, start, end, g, path, avoid);
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

