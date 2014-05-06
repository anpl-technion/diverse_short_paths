/*
 * Frechet.cpp
 */

#include "Frechet.h"

#include "Graph.h"
#include "Path.h"

#define DYN(I,J)    (dynarray[(I)+(J)*rowLength])
#define DIST(I,J)   (si->distance(states1[I], states2[J]))

// Compute Frechet distance between two paths
double Frechet::distance (const Path &path1, const Path &path2)
{
    const ompl::base::SpaceInformationPtr si = path1.getGraph()->getSpaceInfo();
    const std::size_t rowLength = path1.size();
    const std::size_t colLength = path2.size();
    double *const dynarray = new double[rowLength*colLength];
    const std::vector<ompl::base::State *> states1 = path1.getStates();
    const std::vector<ompl::base::State *> states2 = path2.getStates();
    
    DYN(0,0) = 0;
    for (std::size_t i = 1; i < rowLength; i++)
        DYN(i,0) = std::max(DYN(i-1,0), DIST(i,0));
    for (std::size_t j = 1; j < colLength; j++)
        DYN(0,j) = std::max(DYN(0,j-1), DIST(0,j));
    
    for (std::size_t i = 1; i < rowLength; i++)
    {
        for (std::size_t j = 1; j < colLength; j++)
        {
            const double seg = DIST(i,j);
            DYN(i,j) = std::min(
                std::max(DYN(i-1,j), seg),
                std::max(DYN(i,j-1), seg));
        }
    }
    
    const double d = DYN(rowLength-1,colLength-1);
    delete [] dynarray;
    return d;
}

#undef DYN
#undef DIST
