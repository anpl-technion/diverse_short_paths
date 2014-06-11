/*
 * TestData.cpp
 */

#include "TestData.h"

#include "Graph.h"

// Constructors, destructors

TestData::TestData (const std::string &graphFileName, std::size_t numPaths,
    double maxPathLength, double minPathPairwiseDistance)
: k(numPaths), maxLength(maxPathLength), minDistance(minPathPairwiseDistance)
{
    // Read in start, goal, and space information
    start = 0;
    end = 1;
    std::ifstream graphmlstream(graphFileName);
    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(graphmlstream, pt);
    std::string startid = "";
    std::string goalid = "";
    std::string spaceid = "";
    std::string spacetag = "";
    BOOST_FOREACH(boost::property_tree::ptree::value_type const &v, pt.get_child("graphml"))
    {
        if (v.first == "key")
        {
            if (v.second.get<std::string>("<xmlattr>.for") != "graph")
                continue;
            std::string name = v.second.get<std::string>(boost::property_tree::ptree::path_type("<xmlattr>/attr.name", '/'));
            std::string id = v.second.get<std::string>("<xmlattr>.id");
            if (name == "start")
                startid = id;
            else if (name == "goal")
                goalid = id;
            else if (name == "space")
                spaceid = id;
        }
        
        if (v.first == "graph")
        {
            BOOST_FOREACH(boost::property_tree::ptree::value_type const &u, v.second)
            {
                if (u.first != "data")
                    continue;
                std::string key = u.second.get<std::string>("<xmlattr>.key");
                if (key == startid)
                    start = std::atof(u.second.data().c_str());
                else if (key == goalid)
                    end = std::atof(u.second.data().c_str());
                else if (key == spaceid)
                    spacetag = u.second.data();
            }
        }
    }
    
    // Initialize the space according to type
    ompl::base::StateSpacePtr space;
    if (std::strncmp(spacetag.c_str(), "RV", 2) == 0)
    {
        unsigned int dim = std::atoi(&spacetag[2]);
        space = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(dim));
        ompl::base::RealVectorBounds bounds(dim);
        bounds.setLow(-1000);
        bounds.setHigh(1000);
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }
    else if (spacetag == "SE3" || spacetag == "" )    // Default type if not specified
    {
        space = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(-1000);
        bounds.setHigh(1000);
        space->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    }
    else
    {
        std::cerr << "Please add code to handle setup of state space '" << spacetag << "'.\n";
        std::exit(-1);
    }
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    si->setStateValidityChecker([] (const ompl::base::State *) -> bool { return true; });
    si->setup();
    
    graphmlstream.seekg(std::ios_base::beg);
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
