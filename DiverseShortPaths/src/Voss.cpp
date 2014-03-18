/*
 * Voss.cpp
 */

#include "Voss.h"

#include "DStar.h"
#include "Graph.h"
#include "Neighborhood.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

const char *Voss::VOSS_NAME = "Random Neighborhood Avoidance";

Voss::Voss (const TestData *data, double radiusFactor)
  : KDiverseShort(data), radius_factor(radiusFactor)
{
}

const Results *Voss::run ()
{
    std::stringstream desc;
    desc << VOSS_NAME << ":radius_" << radius_factor;
    const Graph &g = testData->getGraph();
    ompl::base::State **statePool = Neighborhood::allocStatePool(g);
    
    if (!needMore())
        return getResults(desc.str().c_str());
    
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<Neighborhood> > resultAvoids;
//     std::vector<DStar *> dstars;
    
    // Unfiltered set of paths found
    std::vector<Path> unfilteredPathSet;
    
    // The next path to analyze
    std::size_t frontier = 0;
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<Neighborhood> alreadyAvoiding;
//     DStar *current_dstar = new DStar(g, testData->getStart(), testData->getEnd());
//     Path referencePath = current_dstar->getPath();
    Path referencePath = getShortestPathUnderAvoidance(alreadyAvoiding);
    if (referencePath.empty())
    {
        Neighborhood::destroyStatePool(g, statePool);
//         delete current_dstar;
        return getResults(desc.str().c_str());
    }

    considerPath(referencePath);
    if (tooLong())
    {
        Neighborhood::destroyStatePool(g, statePool);
//         delete current_dstar;
        return getResults(desc.str().c_str());
    }
    resultAvoids.push_back(alreadyAvoiding);
//     dstars.push_back(current_dstar);
    unfilteredPathSet.push_back(referencePath);
    
    // Work through the queue until we have enough
    while (frontier < unfilteredPathSet.size() && !tooLong() && needMore())
    {
        referencePath = unfilteredPathSet[frontier];
        alreadyAvoiding = resultAvoids[frontier];
//         current_dstar = dstars[frontier];
        frontier++;
        
        // Make attempts at imposing a new neighborhood to avoid on the graph
        // Neighborhood's radius is some pre-defined fraction of the referencePath's length
        // Number of Neighborhoods is twice the number of edges in the path
        double radius = radius_factor * referencePath.getLength();
        for (std::size_t i = 0; i < 2*referencePath.size() && needMore(); i++)
        {
            std::vector<Neighborhood> avoid = alreadyAvoiding;
            avoid.push_back(Neighborhood(g, statePool, referencePath.sampleUniform(), radius));
            // Get the shortest path under these constraints
//             DStar *dstar = new DStar(current_dstar,
//                 Neighborhood(g, statePool, referencePath.sampleUniform(), radius));
//             Path path = dstar->getPath();
            Path path = getShortestPathUnderAvoidance(avoid);
            
            if (path.empty())
                continue;
            
            // Don't store it if it's not diverse enough
            considerPath(path);
            
            // But we'll need it regardless for later iterations
            unfilteredPathSet.push_back(path);
            resultAvoids.push_back(avoid);
//             dstars.push_back(dstar);
        }
    }
    
    Neighborhood::destroyStatePool(g, statePool);
//     for (std::size_t i = 0; i < dstars.size(); i++)
//     {
//         delete dstars[i];
//     }
    return getResults(desc.str().c_str());
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
        for (Vertex v = end;; v = pred[v])
        {
            path.push_back(v);
            if (pred[v] == v)
                break;
        }
        std::reverse(path.begin(), path.end());
    }

    if (path.size() == 1 && start != end)
        path.clear();
    return path;
}

ompl::base::State *Voss::sampleFromPath (const Path &path) const
{
    Vertex v = path[std::rand() % path.size()];
    return testData->getGraph().getVertexState(v);
}
