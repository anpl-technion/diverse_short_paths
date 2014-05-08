/*
 * Levenshtein.cpp
 */

#include "Levenshtein.h"

#include "Graph.h"
#include "Path.h"

// Some convenience macros
#define DYN(I,J)    (dynarray[(I)+(J)*rowLength])
#define DIST(I,J)   (si->distance(states1[I], states2[J]))
#define DIST1(I1,I2)    (si->distance(states1[I1], states1[I2]))
#define DIST2(J1,J2)    (si->distance(states2[J1], states2[J2]))

// Public static methods

double Levenshtein::distance (const Path &path1, const Path &path2)
{
    // Allocate the array and fetch information we'll need
    const ompl::base::SpaceInformationPtr si = path1.getGraph()->getSpaceInfo();
    const std::size_t rowLength = path1.size();
    const std::size_t colLength = path2.size();
    double *const dynarray = new double[rowLength*colLength];
    const std::vector<const ompl::base::State *> states1 = path1.getStates();
    const std::vector<const ompl::base::State *> states2 = path2.getStates();
    
    // Boundary conditions in the array
    DYN(0,0) = DIST(0,0);
    for (std::size_t i = 1; i < rowLength; i++)
        DYN(i,0) = DYN(i-1,0) + DIST(i,0);
    for (std::size_t j = 1; j < colLength; j++)
        DYN(0,j) = DYN(0,j-1) + DIST(0,j);
    
    // Fill the array
    for (std::size_t i = 1; i < rowLength; i++)
    {
        for (std::size_t j = 1; j < colLength; j++)
        {
            const double xform = DYN(i-1,j-1) + DIST(i,j);
//             const double del = DYN(i-1,j) + DIST1(i-1,i) + DIST1(i,i+1) - DIST1(i-1,i+1);
//             const double ins = DYN(i,j-1) + DIST2(j-1,j) + DIST2(j,j+1) - DIST2(j-1,j+1);
            const double del = DYN(i-1,j) + DIST(i,j-1) + DIST(i,j+1) - DIST2(j-1,j+1);
            const double ins = DYN(i,j-1) + DIST(i-1,j) + DIST(i+1,j) - DIST1(i-1,i+1);
            DYN(i,j) = std::min(xform, std::min(ins, del));
        }
    }
    
    // Retrieve the answer
    const double d = DYN(rowLength-1,colLength-1);
    delete [] dynarray;
    return d;
}

#undef DYN
#undef DIST
#undef DIST1
#undef DIST2
