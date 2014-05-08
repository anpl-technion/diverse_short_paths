/*
 * Voss.cpp
 */

#include "Voss.h"

#include "Graph.h"
#include "Neighborhood.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

// Constructors, destructors

Voss::Voss (const TestData *data, double radiusFactor, Neighborhood::AvoidMethod avoid)
: KDiverseShort(data), radiusFactor(radiusFactor), avoidance(avoid)
{
}

// Private methods

std::string Voss::run ()
{
    std::stringstream desc;
    desc << "Random Avoidance, radius factor " << radiusFactor;
    
    // Set up neighborhoods
    const Graph &g = testData->getGraph();
    Neighborhood::setParam(avoidance, &g);
    
    if (!needMore())
        return desc.str();
    
    std::vector<std::vector<Neighborhood> > resultAvoids;   // Avoided neighborhoods each path in resultsPaths was made with
    std::vector<Path> unfilteredPathSet;                    // Unfiltered set of paths found
    std::size_t frontier = 0;                               // The next path to analyze
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<Neighborhood> alreadyAvoiding;
    Path referencePath = getShortestPathUnderAvoidance(alreadyAvoiding);
    if (referencePath.empty())
        return desc.str();
    considerPath(referencePath);
    if (tooLong())
        return desc.str();
    resultAvoids.push_back(alreadyAvoiding);
    unfilteredPathSet.push_back(referencePath);
    
    // Work through the queue until we have enough
    while (frontier < unfilteredPathSet.size() && !tooLong() && needMore())
    {
        referencePath = unfilteredPathSet[frontier];
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make attempts at imposing a new neighborhood to avoid on the graph
        double radius = radiusFactor * referencePath.getLength();
        for (std::size_t i = 0; i < 2 && needMore(); i++)
        {
            std::vector<Neighborhood> avoid = alreadyAvoiding;
            ompl::base::State *sampledState;
            Edge sampledEdge;
            std::tie(sampledState, sampledEdge) = referencePath.sampleUniform();
            avoid.push_back(Neighborhood(sampledState, sampledEdge, radius));
            
            // Get the shortest path under these constraints
            Path path = getShortestPathUnderAvoidance(avoid);
            if (path.empty())
                continue;
            
            // Can we keep it?
            considerPath(path);
            
            // But we'll need it regardless for later iterations
            unfilteredPathSet.push_back(path);
            resultAvoids.push_back(avoid);
        }
    }
    
    return desc.str();
}

Path Voss::getShortestPathUnderAvoidance (const std::vector<Neighborhood> &avoid) const
{
    // Run the A* search
    const Graph &g = testData->getGraph();
    const Vertex start = testData->getStart();
    const Vertex end = testData->getEnd();
    std::vector<Vertex> pred(g.getNumVertices());
    Path path((Graph *)&g);
    try
    {
        boost::astar_search(g, start, heuristic(g, end),
                            boost::weight_map(edgeWeightMap(g, avoid)).
                            predecessor_map(&pred[0]).
                            visitor(visitor(end)));
    }
    catch (foundGoalException e)
    {
        // Trace back the shortest path
        for (Vertex v = end;; v = pred[v])
        {
            path.push_back(v);
            if (pred[v] == v)
                break;
        }
        std::reverse(path.begin(), path.end());
    }
    
    // If no path is found, we want to return an empty path
    if (path.size() == 1 && start != end)
        path.clear();
    
    return path;
}
