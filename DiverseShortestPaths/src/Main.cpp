
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "Graph.h"

int main (int argc, char **argv)
{
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(-10);
    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add 10 random states and connect them randomly
    for (int i = 0; i < 10; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    
    // Make some edges
    g.addEdge(boost::vertex(0, g), boost::vertex(1, g));
    g.addEdge(boost::vertex(0, g), boost::vertex(3, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(4, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(5, g));
    g.addEdge(boost::vertex(3, g), boost::vertex(6, g));
    g.addEdge(boost::vertex(5, g), boost::vertex(6, g));
    g.addEdge(boost::vertex(6, g), boost::vertex(2, g));
    g.addEdge(boost::vertex(6, g), boost::vertex(9, g));
    g.addEdge(boost::vertex(2, g), boost::vertex(7, g));
    g.addEdge(boost::vertex(7, g), boost::vertex(8, g));
    g.addEdge(boost::vertex(9, g), boost::vertex(8, g));
    
    std::vector<Neighborhood> avoid;
    avoid.push_back(Neighborhood(boost::get(boost::vertex_prop, g, boost::vertex(7, g)).state, 0.1));
    std::list<Vertex> path = g.getShortestPathWithAvoidance(boost::vertex(0, g), boost::vertex(8, g), avoid);
    BOOST_FOREACH(Vertex v, path)
    {
        std::cout << v << "\n";
    }
    
    return 0;
}

