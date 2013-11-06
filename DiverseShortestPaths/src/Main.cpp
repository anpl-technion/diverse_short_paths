
/* Author: Caleb Voss */

#include <ctime>
#include <fstream>

#include <boost/foreach.hpp>

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// If NO_DRAW is defined, no output images are produced and cairo library is not required
#ifndef NO_DRAW
#include "Draw.h"
#endif

#include "ActualShortest.h"
#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"
#include "Stats.h"

/** \brief Get a random state from the path (not equal to the start or end of the path)
 * 
 * @tparam N        Neighborhood class to use
 * 
 * @param path      path to sample from
 * @param g         graph this path exists in
 * 
 * @return          pointer to a state on the path
 */
template <class N>
typename N::center_type sampleFromPath (const Path &path, const Graph &g);
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

/** \brief Find a number of diverse, short paths in a graph, by deviating from former short paths through
 *         the use of an increasing set of neighborhoods in the state space that should be avoided
 * 
 * @tparam N                Neighborhood class to use
 * 
 * @param numPaths          number of paths to try to find
 * @param start             start vertex for the path
 * @param end               end vertex for the path
 * @param g                 graph to search in
 * @param radius_factor     neighborhood radius is radius_factor * pathLength
 * @param samples_per_path  number of attempts to generate a new path from an existing one by sampling a neighborhood to avoid
 * @param minDiversity      paths are not retained if their levenshtein edit distance from any already retained path is less than this
 * 
 * @return                  set of at most numPaths distinct paths from start to end
 */
template <class N>
std::vector<Path> findDiverseShortestPaths (const std::size_t numPaths, Vertex start, Vertex end, const Graph &g, const double radius_factor, const std::size_t samples_per_path, const unsigned int minDiversity)
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

Graph *generateGraph(ompl::base::SpaceInformationPtr si, std::size_t n_states, double max_edge_length, double p_connected);

#ifndef NO_DRAW
void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename);
#endif\

int main (int argc, char **argv)
{
    if (argc != 7)
    {
        std::cout << "expected 6 args: <n_trials> <n_states> <n_paths> <radius_factor> <samples_per_path> <minDiversity>\n";
        return 0;
    }
    std::size_t n_trials;
    std::istringstream(argv[1]) >> n_trials;
    std::size_t n_states;
    std::istringstream(argv[2]) >> n_states;
    std::size_t n_paths;
    std::istringstream(argv[3]) >> n_paths;
    double radius_factor;
    std::istringstream(argv[4]) >> radius_factor;
    std::size_t samples_per_path;
    std::istringstream(argv[5]) >> samples_per_path;
    unsigned int minDiversity;
    std::istringstream(argv[6]) >> minDiversity;
    
    // Tweakable params for graph generation
    const double max_edge_length = 4.0/std::sqrt(n_states/100.0);
    const double p_connected = 0.25;
    
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    
    std::stringstream filename;
    filename << "test_" << n_states << "_" << n_paths << "_" << radius_factor << "_" << samples_per_path<< "_" << minDiversity << ".txt";
    std::ofstream file(filename.str().c_str(), std::ofstream::out | std::ofstream::trunc);
    
    file << "GRAPH GENERATION:\n";
    file << " number of states: " << n_states << "\n";
    file << " maximum edge length: " << max_edge_length << "\n";
    file << " P(edge between u,v | d(u,v) < maximum edge length): " << p_connected << "\n\n";
    
    file << "PROBLEM SPECIFICATION:\n";
    file << " find " << n_paths << " paths subject to the constraint that\n";
    file << " all pairs of paths have Levenshtein edit distance at least " << minDiversity << "\n\n\n";
    
    std::vector<PathSetStats> statsExhaustive;
    std::vector<PathSetStats> statsSpace;
    std::vector<PathSetStats> statsGraph;
    std::cout << "Running " << n_trials << " trials for each algorithm:\n";
    for (std::size_t i = 0; i < n_trials; i++)
    {
        Graph *g = generateGraph(si, n_states, max_edge_length, p_connected);
        Vertex start = boost::vertex(0, *g);
        Vertex goal = boost::vertex(1, *g);
        
        std::cout << "Exhaustive:\n";
        double runtime = std::clock();
        std::vector<Path> paths = actualShortestPaths(n_paths, *g, start, goal, minDiversity);
        runtime = std::clock()-runtime;
        statsExhaustive.push_back(PathSetStats(paths, runtime/CLOCKS_PER_SEC));
        
        std::cout << "Space:\n";
        runtime = std::clock();
        std::vector<Path> paths2 = findDiverseShortestPaths<StateSpaceNeighborhood>(n_paths, start, goal, *g, radius_factor, samples_per_path, minDiversity);
        runtime = std::clock()-runtime;
        statsSpace.push_back(PathSetStats(paths2, runtime/CLOCKS_PER_SEC));
        
        std::cout << "Graph:\n";
        runtime = std::clock();
        std::vector<Path> paths3 = findDiverseShortestPaths<GraphDistanceNeighborhood>(n_paths, start, goal, *g, radius_factor, samples_per_path, minDiversity);
        runtime = std::clock()-runtime;
        statsGraph.push_back(PathSetStats(paths3, runtime/CLOCKS_PER_SEC));
        
        std::cout << " completed trial " << i+1 << "/" << n_trials << "\n\n";
        
#ifndef NO_DRAW
        if (i = n_trials-1)
        {
            draw(*g, start, goal, paths, "output_space.png");
            draw(*g, start, goal, paths2, "output_graph.png");
            draw(*g, start, goal, paths3, "output_exhaustive.png");
        }
#endif
    
        delete g;
    }
    
    file << "All results are computed with respect to the data from the exhaustive search with greedy filtering algorithm.\n\n";
    
    file << "ALGORITHM: Stochastic avoidance sampling using space metric\n";
    file << " (neighborhood radius: " << radius_factor << " * path length)\n";
    file << " (samples per path: " << samples_per_path << ")\n";
    AlgorithmSummary(statsSpace, statsExhaustive).print(file);
    
    file << "ALGORITHM: Stochastic avoidance sampling using graph distance\n";
    file << " (neighborhood radius: " << radius_factor << " * path length)\n";
    file << " (samples per path: " << samples_per_path << ")\n";
    AlgorithmSummary(statsGraph, statsExhaustive).print(file);
    
    file.close();
    
    return 0;
}

Graph *generateGraph(ompl::base::SpaceInformationPtr si, std::size_t n_states, double max_edge_length, double p_connected)
{
    Graph *g = new Graph(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add n_states random states and connect them randomly
    for (std::size_t i = 0; i < n_states; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g->addVertex(state);
    }
    for (std::size_t i = 0; i < n_states; i++)
    {
        for (std::size_t j = 0; j < n_states; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, *g, boost::vertex(i, *g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, *g, boost::vertex(j, *g)).state;
            if (si->getStateSpace()->distance(iState, jState) <= max_edge_length)
            {
                if (float(std::rand())/RAND_MAX < p_connected)
                    g->addEdge(boost::vertex(i, *g), boost::vertex(j, *g));
            }
        }
    }
    
    return g;
}


