/*
 * TestData.cpp
 */

#include "TestData.h"

#include "Graph.h"

// Constructors, destructors

TestData::TestData (const std::string &graphFileName, std::size_t numPaths,
    double maxPathLength, double minPathPairwiseDistance)
:  start(0), end(1), k(numPaths), maxLength(maxPathLength), minDistance(minPathPairwiseDistance)
{
    // Initialize the space
    ompl::base::SpaceInformationPtr si;
    {
        ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-1000);
        bounds.setHigh(1000);
        space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
        si = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(space));
        si->setup();
    }
    
    std::ifstream graphmlstream(graphFileName);
    graph = new Graph(si, graphmlstream);
}

TestData::~TestData ()
{
    delete graph;
}

// Public methods

Vertex TestData::getStart () const
{
    return start;
}

Vertex TestData::getEnd () const
{
    return end;
}

std::size_t TestData::getK () const
{
    return k;
}

double TestData::getMaxLength () const
{
    return maxLength;
}

double TestData::getMinDistance () const
{
    return minDistance;
}

const Graph &TestData::getGraph () const
{
    return *graph;
}
