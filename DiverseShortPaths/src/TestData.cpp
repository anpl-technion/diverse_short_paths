/*
 * TestData.cpp
 */

#include "TestData.h"

#include "Graph.h"

extern const double maxPathLength;
extern const double minPathPairwiseDistance;

TestData::TestData ()
 :  graph(NULL), start(0), end(0), maxLength(maxPathLength), minDistance(minPathPairwiseDistance), k(0)
{ }

TestData::~TestData ()
{
    delete graph;
}

TestData *TestData::generate (const char *graphFileName, const std::size_t k)
{
    TestData *data = new TestData();
    
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
    data->setGraph(new Graph(si, graphmlstream));
    data->setStartEnd(0, 1);
    data->setK(k);
    
    return data;
}

void TestData::setGraph (const Graph *g)
{
    graph = g;
}

const Graph &TestData::getGraph () const
{
    return *graph;
}

void TestData::setStartEnd (const Vertex s, const Vertex e)
{
    start = s;
    end = e;
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

void TestData::setK (const std::size_t newK)
{
    k = newK;
}

double TestData::getK () const
{
    return k;
}
