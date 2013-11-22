
/* Author: Caleb Voss */

#ifndef EXHAUSTIVE_H
#define EXHAUSTIVE_H

#include <queue>

#include <boost/function.hpp>

#include "Common.h"

typedef std::pair<Path, std::vector<SingleEdgeNeighborhood> > PathAndAvoidance;

bool comparePaths (const Path &path1, const Path &path2);

bool compareMutablePaths (const MutablePath &path1, const MutablePath &path2);
typedef boost::function<bool(const MutablePath&, const MutablePath&)> f_compareMutablePaths;

bool rcomparePathAndAvoidances (const PathAndAvoidance &pa1, const PathAndAvoidance &pa2);
typedef boost::function<bool(const PathAndAvoidance&, const PathAndAvoidance&)> f_comparePathAndAvoidances;

std::vector<Path> exhaustiveShortestPaths (const std::size_t numPaths, const Graph &g, Vertex start, Vertex end, const unsigned int minDiversity);\

#endif
