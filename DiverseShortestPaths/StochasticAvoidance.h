
/* Author: Caleb Voss */

#ifndef STOCHASTIC_AVOIDANCE_H
#define STOCHASTIC_AVOIDANCE_H

#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"

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
ompl::base::State *sampleFromPath <StateSpaceNeighborhood> (const Path &path, const Graph &g);

template <>
Vertex sampleFromPath <GraphDistanceNeighborhood> (const Path &path, const Graph &g);

/** \brief Templated function that actually runs the algorithm. See next function.
 */
template <class N>
std::vector<Path> stochasticAvoidancePathsT (const std::size_t numPaths, Vertex start, Vertex end, const Graph &g, const double radius_factor,
                                            const std::size_t samples_per_path, const unsigned int minDiversity);

/** \brief Find a number of diverse, short paths in a graph, by deviating from former short paths through
 *         the use of an increasing set of neighborhoods in the state space that should be avoided
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
std::vector<Path> stochasticAvoidancePaths (bool nbh_type, const std::size_t numPaths, Vertex start, Vertex end, const Graph &g,
                                            const double radius_factor, const std::size_t samples_per_path, const unsigned int minDiversity);

#endif

