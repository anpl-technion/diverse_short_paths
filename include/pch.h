/*
 * pch.h
 */

#ifndef __PCH_H
#define __PCH_H

// Standard Library

#include <algorithm>
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// BOOST

#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphml.hpp>

// OMPL

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>

// Class Prototypes

class Eppstein;
class Frechet;
class Graph;
class KDiverseShort;
class Levenshtein;
class Neighborhood;
class Path;
class PathDistanceMeasure;
class Results;
class TestData;
class Voss;
namespace graehl { class Graehl; }

#endif
