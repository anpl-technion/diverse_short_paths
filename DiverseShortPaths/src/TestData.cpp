/*
 * TestData.cpp
 */

#include "TestData.h"

#include "Graph.h"

TestData::TestData (const std::string &graphFileName, const std::size_t numPaths,
                    const double maxPathLength, const double minPathPairwiseDistance)
 :  start(0), end(1), k(numPaths), maxLength(maxPathLength), minDistance(minPathPairwiseDistance), mode(Mode::UNSET), graphName(graphFileName)
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
    
    // TODO: Remove connected components not containing start state
}

TestData::~TestData ()
{
    delete graph;
}

const std::string TestData::getName () const
{
    return graphName;
}

void TestData::setMode (Mode m)
{
    mode = m;
}

const Graph &TestData::getGraph () const
{
    return *graph;
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
    switch (mode)
    {
    case FIX_MAX_PATH_LENGTH:
        return maxLength;
    case FIX_MIN_PATH_DISTANCE:
        return std::numeric_limits<double>::infinity();
    default:
        throw std::runtime_error("Test mode not set!");
    }
}

double TestData::getMinDistance () const
{
    switch (mode)
    {
    case FIX_MAX_PATH_LENGTH:
        return 1e-12;
    case FIX_MIN_PATH_DISTANCE:
        return minDistance;
    default:
        throw std::runtime_error("Test mode not set!");
    }
}

std::string TestData::getParameter () const
{
    std::stringstream param;
    switch (mode)
    {
        case FIX_MAX_PATH_LENGTH:
            param << "length_" << maxLength;
            break;
        case FIX_MIN_PATH_DISTANCE:
            param << "dist_" << minDistance;
            break;
        default:
            throw std::runtime_error("Test mode not set!");
    }
    
    return param.str();
}

double TestData::getK () const
{
    return k;
}
