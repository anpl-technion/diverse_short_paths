
/* Author: Caleb Voss */

#include "StochasticAvoidance.h"

#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"

template <>
ompl::base::State *sampleFromPath <StateSpaceNeighborhood> (const Path &path, const Graph &g)
{
    return path.sampleState(g);
}
template <>
Vertex sampleFromPath <GraphDistanceNeighborhood> (const Path &path, const Graph &g)
{
    return path.sampleVertex();
}

template <class N>
std::vector<Path> stochasticAvoidancePathsT (const std::size_t numPaths, Vertex start, Vertex end, const Graph &g, const double radius_factor,
                                             const std::size_t samples_per_path, const unsigned int minDiversity)
{
    // Holds the set of paths we've found
    std::vector<Path> resultPaths;
    if (numPaths == 0)
        return resultPaths;
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<N> > resultAvoids;
    // The next path to analyze
    std::size_t frontier = 0;
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<N> alreadyAvoiding = std::vector<N>();
    Path referencePath = g.getShortestPathWithAvoidance<N>(start, end, alreadyAvoiding);
    if (referencePath.empty())
    {
        std::cout << "done!\n";
        return resultPaths;
    }
    
    resultPaths.push_back(referencePath);
    resultAvoids.push_back(alreadyAvoiding);
    std::cout << "Kept: " << resultPaths.size() << "/" << numPaths << "\n";
    
    // Work through the queue until we have enough
    while (frontier < resultPaths.size() && resultPaths.size() < numPaths)
    {
        referencePath = resultPaths[frontier];
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make attempts at imposing a new neighborhood to avoid on the graph
        // Neighborhood's radius is some pre-defined portion of the referencePath's length
        double radius = radius_factor * referencePath.getLength();
        for (int i = 0; i < samples_per_path && resultPaths.size() < numPaths; i++)
        {
            if (!referencePath.sampleable())
                break;
            std::vector<N> avoid = alreadyAvoiding;
            avoid.push_back(N(g, sampleFromPath<N>(referencePath, g), radius));
            // Get the shortest path under these constraints
            Path path = g.getShortestPathWithAvoidance<N>(start, end, avoid);
            if (path.empty())
                continue;
            
            // Don't store it if it's not diverse enough
            bool tooSimilar = false;
            BOOST_FOREACH(Path p, resultPaths)
            {
                if (Path::levenshtein(path, p) < minDiversity)
                {
                    tooSimilar = true;
                    break;
                }
            }
            if (!tooSimilar)
            {
                resultPaths.push_back(path);
                resultAvoids.push_back(avoid);
                std::cout << "Kept: " << resultPaths.size() << "/" << numPaths << "\n";
            }
        }
    }
    
    std::cout << "done!\n";
    return resultPaths;
}

std::vector<Path> stochasticAvoidancePaths (bool nbh_type, const std::size_t numPaths, Vertex start, Vertex end, const Graph &g,
                                            const double radius_factor, const std::size_t samples_per_path, const unsigned int minDiversity)
{
    if (nbh_type == NBH_STATESPACE)
        return stochasticAvoidancePathsT<StateSpaceNeighborhood>(numPaths, start, end, g, radius_factor, samples_per_path, minDiversity);
    else
        return stochasticAvoidancePathsT<GraphDistanceNeighborhood>(numPaths, start, end, g, radius_factor, samples_per_path, minDiversity);
}

