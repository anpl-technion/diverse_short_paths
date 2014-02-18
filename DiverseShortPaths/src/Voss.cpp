/*
 * Voss.cpp
 */

#include "Voss.h"

#include "Graph.h"
#include "Neighborhood.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

Voss::Voss (const TestData *data)
  : KDiverseShort(data)
{ }

const double radius_factor = 0.05;
const std::size_t samples_per_path = 5;

const Results *Voss::run ()
{
    const Graph &g = testData->getGraph();
    
    if (!needMore())
        return getResults();
    
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<Neighborhood> > resultAvoids;
    
    // The next path to analyze
    std::size_t frontier = 0;
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<Neighborhood> alreadyAvoiding;
    Path referencePath = getShortestPathUnderAvoidance(alreadyAvoiding);
    if (referencePath.empty())
        return getResults();

    considerPath(referencePath);
    if (tooLong())
        return getResults();
    resultAvoids.push_back(alreadyAvoiding);
    std::cout << "Kept: " << 1 << "/" << testData->getK() << "\n";
    
    // Work through the queue until we have enough
    while (frontier < pathSet.size() && !tooLong() && needMore())
    {
        referencePath = pathSet[frontier];
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make attempts at imposing a new neighborhood to avoid on the graph
        // Neighborhood's radius is some pre-defined portion of the referencePath's length
        double radius = radius_factor * referencePath.getLength();
        for (std::size_t i = 0; i < samples_per_path && needMore(); i++)
        {
            std::vector<Neighborhood> avoid = alreadyAvoiding;
            avoid.push_back(Neighborhood(g, sampleFromPath(referencePath), radius));
            // Get the shortest path under these constraints
            Path path = getShortestPathUnderAvoidance(avoid);
            if (path.empty())
                continue;
            
            // Don't store it if it's not diverse enough
            if (considerPath(path))
            {
                resultAvoids.push_back(avoid);
                std::cout << "Kept: " << pathSet.size() << "/" << testData->getK() << "\n";
            }
        }
    }
    
    BOOST_FOREACH(Path path, pathSet)
    {
        path.print();
    }
    std::cout << "\n\n";
    return getResults();
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
