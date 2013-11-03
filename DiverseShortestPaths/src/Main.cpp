
/* Author: Caleb Voss */

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

#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"
#include "ActualShortest.h"

// !!! This could be improved by asking for n samples at once without replacement
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
    Path::const_iterator vi = path.begin();
    for (std::size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return boost::get(boost::vertex_prop, g, *vi).state;
}
template <>
Vertex sampleFromPath <GraphDistanceNeighborhood> (const Path &path, const Graph &g)
{
    Path::const_iterator vi = path.begin();
    for (std::size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return *vi;
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
        return resultPaths;
    
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
    
    return resultPaths;
}

#ifndef NO_DRAW
void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename);
#endif
void printStats (Graph &g, std::vector<Path> &paths);

int main (int argc, char **argv)
{
    if (argc != 6)
    {
        std::cout << "expected 5 args: <n_states> <n_paths> <radius_factor> <samples_per_path> <minDiversity>\n";
        return 0;
    }
    std::size_t n_states;
    std::istringstream(argv[1]) >> n_states;
    std::size_t n_paths;
    std::istringstream(argv[2]) >> n_paths;
    double radius_factor;
    std::istringstream(argv[3]) >> radius_factor;
    std::size_t samples_per_path;
    std::istringstream(argv[4]) >> samples_per_path;
    unsigned int minDiversity;
    std::istringstream(argv[5]) >> minDiversity;
    
    // Tweakable params for graph generation
    const double max_edge_length = 4.0/std::sqrt(n_states/100.0);
    const double p_connected = 0.25;
    
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add n_states random states and connect them randomly
    for (std::size_t i = 0; i < n_states; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    for (std::size_t i = 0; i < n_states; i++)
    {
        for (std::size_t j = 0; j < n_states; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, g, boost::vertex(i, g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, g, boost::vertex(j, g)).state;
            if (space->distance(iState, jState) <= max_edge_length)
            {
                if (float(std::rand())/RAND_MAX < p_connected)
                    g.addEdge(boost::vertex(i, g), boost::vertex(j, g));
            }
        }
    }
    
    Vertex start = boost::vertex(0, g);
    Vertex goal = boost::vertex(8, g);
    std::cout << "Finding at most " << n_paths << " diverse short paths from node " << start << " to node " << goal << "\n\n";
    std::vector<Path> paths = findDiverseShortestPaths<StateSpaceNeighborhood>(n_paths, start, goal, g, radius_factor, samples_per_path, minDiversity);
    std::cout << "Completed space distance version\n\n";
    std::vector<Path> paths2 = findDiverseShortestPaths<GraphDistanceNeighborhood>(n_paths, start, goal, g, radius_factor, samples_per_path, minDiversity);
    std::cout << "Completed graph distance version\n\n";
    std::vector<Path> paths3 = actualShortestPaths(n_paths, g, start, goal, minDiversity);
    std::cout << "Completed actual shortest paths\n\n";
    std::cout << "Got " << paths.size() << ", " << paths2.size() << ", " << paths3.size() << " paths\n";
    
#ifndef NO_DRAW
    draw(g, start, goal, paths, "output_space.png");
    draw(g, start, goal, paths2, "output_graph.png");
    draw(g, start, goal, paths3, "output_actual.png");
#endif
    
    if (paths.size() > 1)
    {
        std::cout << "\nSeparation of paths with state space neighborhoods:\n";
        printStats(g, paths);
    }
    if (paths2.size() > 1)
    {
        std::cout << "\nSeparation of paths with graph distance neighborhoods:\n";
        printStats(g, paths2);
    }
    if (paths3.size() > 1)
    {
        std::cout << "\nSeparation of the actual shortest paths:\n";
        printStats(g, paths3);
    }
    
    return 0;
}

void printStats (Graph &g, std::vector<Path> &paths)
{
    // Compute Levenshtein edit distance between every pair as a diversity measure
    unsigned int min = std::numeric_limits<unsigned int>::max();
    unsigned int max = 0;
    unsigned int total = 0;
    unsigned int count = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1 == path2)
                continue;
            unsigned int distance = Path::levenshtein(path1, path2);
            if (distance < min)
                min = distance;
            if (distance > max)
                max = distance;
            total += distance;
            count++;
        }
    }
    
    // Report min, max, and mean
    std::cout << "DISTANCE BETWEEN PATHS: min: " << min << ", max: " << max << ", mean: " << double(total)/count << "\n";
    
    // Compute Levenshtein edit distance between every path and it's nearest neighbor
    min = std::numeric_limits<unsigned int>::max();
    total = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        Path nearest(path1);
        unsigned int distance = 0;
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1 == path2)
                continue;
            distance = Path::levenshtein(path1, path2);
            if (distance < min)
                min = distance;
        }
        total += distance;
    }
    
    // Report min, max, and mean
    std::cout << "DISTANCE TO NEAREST NEIGHBOR: mean: " << double(total)/paths.size() << "\n";
    
    // Compute length of every path
    double lmin = std::numeric_limits<double>::max();
    double lmax = 0;
    double ltotal = 0;
    BOOST_FOREACH(Path path, paths)
    {
        double distance = g.pathLength(path);
        if (distance < lmin)
            lmin = distance;
        if (distance > max)
            lmax = distance;
        ltotal += distance;
    }
    
    // Report min, max, and mean
    std::cout << "LENGTH OF PATHS: min: " << lmin << ", max: " << lmax << ", mean: " << double(ltotal)/paths.size() << "\n";
}

// Compare with via-point method
