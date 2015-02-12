/*
 * Hausdorff.cpp
 */

#include "Hausdorff.h"

#include "Graph.h"
#include "Path.h"

// Some convenient macros
#define DYN(I,J)    (distarray[(I)+(J)*rowLength])
#define DIST(a,b)   (si->distance(a, b))
#define MINDIST(s1,e1,s2,e2) \
  std::min(std::min(DIST(s1,s2), DIST(s1,e2)), std::min(DIST(e1,s2), DIST(e1,e2)))

// Public methods

std::string Hausdorff::getName ()
{
    return "Hausdorff";
}

double Hausdorff::distance (const Path &path1, const Path &path2)
{
  const ompl::base::SpaceInformationPtr si = path1.getGraph()->getSpaceInfo();
  const std::size_t rowLength = path1.size();
  const std::size_t colLength = path2.size();
  double *const distarray = new double[rowLength*colLength];
  const std::vector<const ompl::base::State *> states1 = path1.getStates();
  const std::vector<const ompl::base::State *> states2 = path2.getStates();

  // Initialize the min records.
  for (std::size_t i = 0; i < rowLength-1; i++)
    DYN(i,colLength-1) = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < colLength-1; j++)
    DYN(rowLength-1,j) = std::numeric_limits<double>::infinity();

  // Fill in the distance matrix.
  for (std::size_t i = 0; i < rowLength-1; i++)
  {
    for (std::size_t j = 0; j < colLength-1; j++)
    {
      DYN(i,j) = MINDIST(states1[i], states1[i+1], states2[j], states2[j+1]);

      // Update the min records.
      DYN(i,colLength-1) = std::min(DYN(i,colLength-1), DYN(i,j));
      DYN(rowLength-1,j) = std::min(DYN(rowLength-1,j), DYN(i,j));
    }
  }

  // Compute the max over both min records.
  double max = 0;
  for (std::size_t i = 0; i < rowLength-1; i++)
    max = std::max(max, DYN(i,colLength-1));
  for (std::size_t j = 0; j < colLength-1; j++)
    max = std::max(max, DYN(rowLength-1,j));

  delete [] distarray;

  return max;
}

#undef DIST
