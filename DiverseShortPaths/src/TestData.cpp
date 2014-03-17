/*
 * TestData.cpp
 */

#include "TestData.h"

#include "Graph.h"

TestData::TestData (const std::string &graphFileName, const std::size_t numPaths,
                    const double maxPathLength, const double minPathPairwiseDistance)
 :  start(0), end(1), k(numPaths), maxLength(maxPathLength), minDistance(minPathPairwiseDistance), graphName(graphFileName)
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
    
    buf = new char[graphName.length() + 1];
    
    // TODO: Remove connected components not containing start state
}

TestData::~TestData ()
{
    delete [] buf;
    delete graph;
}

const Graph &TestData::getGraph () const
{
    return *graph;
}

char *TestData::getName () const
{
    graphName.copy(buf, graphName.length());
    buf[graphName.length()] = '\0';
    return basename(buf);
}

Vertex TestData::getStart () const
{
    return start;
}

Vertex TestData::getEnd () const
{
    return end;
}

double TestData::getMaxLength () const
{
    return maxLength;
}

double TestData::getMinDistance () const
{
    return minDistance;
}

double TestData::getK () const
{
    return k;
}
