
/* Author: Caleb Voss */

#ifndef ACTUAL_SHORTEST_H_
#define ACTUAL_SHORTEST_H_

#include <queue>

#include "Graph.h"
#include "Path.h"

struct PathAndAvoidance
{
    Path path;
    std::vector<SingleEdgeNeighborhood> avoids;
    
    PathAndAvoidance (Path path, std::vector<SingleEdgeNeighborhood> avoids)
    : path(path), avoids(avoids)
    {
    }
};

bool comparePaths (Graph &g, Path path1, Path path2)
{
    return path1.getLength() < path2.getLength();
}
typedef boost::function<bool(Path, Path)> f_comparePaths;

bool comparePathAndAvoidances (Graph &g, PathAndAvoidance pa1, PathAndAvoidance pa2)
{
    return comparePaths(g, pa1.path, pa2.path);
}
typedef boost::function<bool(PathAndAvoidance, PathAndAvoidance)> f_comparePathAndAvoidances;

std::vector<Path> &actualShortestPaths (size_t numPaths, Graph &g, Vertex start, Vertex end, unsigned int minDiversity)
{
    f_comparePaths comp1 = boost::bind(comparePaths, g, _1, _2);
    f_comparePathAndAvoidances comp2 = boost::bind(comparePathAndAvoidances, g, _2, _1);
    // Holds the shortest paths we've found and finished with
    std::set<Path, f_comparePaths> resultPaths(comp1);
    // How many of the paths in resultPaths are certainly the top shortest, and an iterator to the longest such path
    size_t nCertainPaths = 0;
    std::set<Path, f_comparePaths>::reverse_iterator longestCertain = resultPaths.rend();
    size_t nThrownOutPaths = 0;
    // Holds the shortest paths we've found, but still need to use
    std::priority_queue<PathAndAvoidance, std::vector<PathAndAvoidance>, f_comparePathAndAvoidances> frontier(comp2);
    
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<SingleEdgeNeighborhood> alreadyAvoiding = std::vector<SingleEdgeNeighborhood>();
    Path referencePath = g.getShortestPathWithAvoidance<SingleEdgeNeighborhood>(start, end, alreadyAvoiding);
    if (referencePath.getPath().empty())
        return *new std::vector<Path>();
    
    // Seed the frontier
    frontier.push(PathAndAvoidance(referencePath, alreadyAvoiding));
    
    while (!frontier.empty())
    {
        PathAndAvoidance pa = frontier.top();
        frontier.pop();
        referencePath = pa.path;
        alreadyAvoiding = pa.avoids;
        
        // Make a pre-defined number of attempts at imposing a new neighborhood to avoid on the graph
        Path::iterator i = referencePath.begin();
        i++;
        for (Path::iterator j = referencePath.begin(); i != referencePath.end(); i++, j++)
        {
            std::vector<SingleEdgeNeighborhood> avoid = alreadyAvoiding;
            avoid.push_back(SingleEdgeNeighborhood(boost::edge(*j, *i, g).first));
            // Get the shortest path under these constraints
            Path path = g.getShortestPathWithAvoidance<SingleEdgeNeighborhood>(start, end, avoid);
            if (path.getPath().empty())
                continue;
            
            // Don't store it if it's a duplicate
            bool alreadyHave = false;
            BOOST_FOREACH(Path p, resultPaths)
            {
                if (path.getPath() == p.getPath())
                {
                    alreadyHave = true;
                    break;
                }
            }
            if (!alreadyHave)
                frontier.push(PathAndAvoidance(path, avoid));
        }
        // Save the reference path
        resultPaths.insert(referencePath);
        if (longestCertain != resultPaths.rend() && longestCertain->getPath() == referencePath.getPath())
            longestCertain--;
        
        // Check the path after the longest certain to see if we can stop
        longestCertain--;
        if (!frontier.empty() && comparePaths(g, *longestCertain, frontier.top().path))
        {
            nCertainPaths++;
            
            // Check if longestCertain is different enough from existing paths
            for (std::set<Path, f_comparePaths>::reverse_iterator keptPath = longestCertain; keptPath != resultPaths.rend(); keptPath++)
            {
                if (keptPath == longestCertain || keptPath->isMarkedForDeletion())
                    continue;
                unsigned int distance = levenshtein(keptPath->getPath(), longestCertain->getPath());
                if (distance < minDiversity)
                {
                    nThrownOutPaths++;
                    longestCertain->markForDeletion();
                    break;
                }
            }
            
            std::cout << "Certain about " << nCertainPaths << " paths; kept " << nCertainPaths-nThrownOutPaths << "\n";
            if (nCertainPaths-nThrownOutPaths == numPaths)
                break;
        }
        else
        {
            longestCertain++;
        }
    }
    
    // Only return what we need
    std::vector<Path> *ret = new std::vector<Path>();
    std::set<Path, f_comparePaths>::iterator path = resultPaths.begin();
    for (size_t i = 0; i < numPaths && path != resultPaths.end(); path++)
    {
        if (!path->isMarkedForDeletion())
        {
            ret->push_back(*path);
            i++;
        }
    }
    return *ret;
}

#endif
