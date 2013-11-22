
/* Author: Caleb Voss */

#include "Exhaustive.h"

#include <queue>

#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"

bool comparePaths (const Path &path1, const Path &path2)
{
    return path1.getLength() < path2.getLength();
}

bool compareMutablePaths (const MutablePath &path1, const MutablePath &path2)
{
    return path1.path.getLength() < path2.path.getLength();
}

bool rcomparePathAndAvoidances (const PathAndAvoidance &pa1, const PathAndAvoidance &pa2)
{
    return comparePaths(pa2.first, pa1.first);
}

std::vector<Path> exhaustiveShortestPaths (const std::size_t numPaths, const Graph &g, Vertex start, Vertex end, const unsigned int minDiversity)
{
    std::vector<Path> ret;;
    // Holds the shortest paths we've found and finished with
    std::set<MutablePath, f_compareMutablePaths> resultPaths(compareMutablePaths);
    // How many of the paths in resultPaths are certainly the top shortest, and an iterator to the longest such path
    std::size_t nCertainPaths = 0;
    std::set<MutablePath, f_compareMutablePaths>::iterator longestCertain = resultPaths.begin();
    std::size_t nThrownOutPaths = 0;
    // Holds the shortest paths we've found, but still need to use
    std::priority_queue<PathAndAvoidance, std::vector<PathAndAvoidance>, f_comparePathAndAvoidances> frontier(rcomparePathAndAvoidances);
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<SingleEdgeNeighborhood> alreadyAvoiding = std::vector<SingleEdgeNeighborhood>();
    Path referencePath = g.getShortestPathWithAvoidance<SingleEdgeNeighborhood>(start, end, alreadyAvoiding);
    if (referencePath.empty())
    {
        std::cout << "done!\n";
        return ret;
    }
    
    // Seed the frontier
    frontier.push(PathAndAvoidance(referencePath, alreadyAvoiding));
    
    // Unfortunately, we need to stop if the frontier gets too large
    while (!frontier.empty() && frontier.size() < 3000000)
    {
        referencePath = frontier.top().first;
        alreadyAvoiding = frontier.top().second;
        frontier.pop();
        
        // Make a pre-defined number of attempts at imposing a new neighborhood to avoid on the graph
        Path::iterator i = referencePath.begin();
        i++;
        for (Path::iterator j = referencePath.begin(); i != referencePath.end(); i++, j++)
        {
            std::vector<SingleEdgeNeighborhood> avoid = alreadyAvoiding;
            avoid.push_back(SingleEdgeNeighborhood(boost::edge(*j, *i, g).first));
            // Get the shortest path under these constraints
            Path path = g.getShortestPathWithAvoidance<SingleEdgeNeighborhood>(start, end, avoid);
            if (path.empty())
                continue;
            
            // Don't store it if it's a duplicate
            bool alreadyHave = false;
            BOOST_FOREACH(MutablePath p, resultPaths)
            {
                if (path == p.path)
                {
                    alreadyHave = true;
                    break;
                }
            }
            if (!alreadyHave)
                frontier.push(PathAndAvoidance(path, avoid));
        }
        // Save the reference path
        resultPaths.insert(MutablePath(referencePath));
        
        // Check the path after the longest certain to see if we can stop
        if (longestCertain == resultPaths.end())
        {
            // This should only happen the first time when longestCertain was already pointing to the end
            longestCertain--;
        }
        else
        {
            longestCertain++;
            // longestcertain might end up pointing to the path we just added accidentally 
            if (longestCertain == resultPaths.end())
            {
                longestCertain--;
            }
        }
        if (!frontier.empty() && comparePaths(longestCertain->path, frontier.top().first))
        {
            nCertainPaths++;
            
            // Check if the candidate is different enough from existing paths
            for (std::set<MutablePath, f_compareMutablePaths>::iterator keptPath = resultPaths.begin(); keptPath != longestCertain; keptPath++)
            {
                if (keptPath->isFreed())
                    continue;
                unsigned int distance = Path::levenshtein(keptPath->path, longestCertain->path);
                if (distance < minDiversity)
                {
                    nThrownOutPaths++;
                    longestCertain->free();
                    break;
                }
            }
            
            std::cout << "Certain: " << nCertainPaths << "; Kept: " << nCertainPaths-nThrownOutPaths << "/" << numPaths << "\n";
            if (nCertainPaths-nThrownOutPaths == numPaths)
                break;
        }
        else
        {
            longestCertain--;
        }
    }
    
    if (frontier.size() >= 3000000)
        std::cout << "Frontier exceeded 3 million; stopping prematurely.\n";
        
    // Only return what we need
    std::set<MutablePath, f_compareMutablePaths>::iterator path = resultPaths.begin();
    for (std::size_t i = 0; i < numPaths && path != resultPaths.end(); path++)
    {
        if (!path->isFreed())
        {
            ret.push_back(path->path);
            i++;
        }
    }
    std::cout << "done!\n";
    return ret;
}

