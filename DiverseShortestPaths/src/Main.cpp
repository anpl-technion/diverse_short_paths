
/* Author: Caleb Voss */

#include <ctime>
#include <fstream>

#include <boost/foreach.hpp>

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// If NO_DRAW is defined, no output images are produced and cairo library is not required
#ifndef NO_DRAW
#include "Draw.h"
#endif

#include "Common.h"
#include "Exhaustive.h"
#include "Graph.h"
#include "Neighborhoods.h"
#include "Path.h"
#include "Stats.h"
#include "StochasticAvoidance.h"

Graph *generateGraph(ompl::base::SpaceInformationPtr si, std::size_t n_states, double max_edge_length, double p_connected);

#ifndef NO_DRAW
void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename);
#endif\

int main (int argc, char **argv)
{
    if (argc != 7)
    {
        std::cout << "expected 6 args: <n_trials> <n_states> <n_paths> <radius_factor> <samples_per_path> <minDiversity>\n";
        return 0;
    }
    std::size_t n_trials;
    std::istringstream(argv[1]) >> n_trials;
    std::size_t n_states;
    std::istringstream(argv[2]) >> n_states;
    std::size_t n_paths;
    std::istringstream(argv[3]) >> n_paths;
    double radius_factor;
    std::istringstream(argv[4]) >> radius_factor;
    std::size_t samples_per_path;
    std::istringstream(argv[5]) >> samples_per_path;
    unsigned int minDiversity;
    std::istringstream(argv[6]) >> minDiversity;
    
    // Tweakable params for graph generation
    const double max_edge_length = 4.0/std::sqrt(n_states/100.0);
    const double p_connected = 0.25;
    
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    
    if (n_trials == 0)
    {
        Graph *g = generateGraph(si, n_states, max_edge_length, p_connected);
        Vertex start = boost::vertex(0, *g);
        Vertex goal = boost::vertex(1, *g);
        
        std::vector<Path> paths = stochasticAvoidancePaths(NBH_STATESPACE, n_paths, start, goal, *g, radius_factor, samples_per_path, minDiversity);
        
#ifndef NO_DRAW
        draw(*g, start, goal, paths, "output_space.png");
#endif
        
        delete g;
        return 0;
    }
    
    std::stringstream filename;
    filename << "test_" << n_states << "_" << n_paths << "_" << radius_factor << "_" << samples_per_path<< "_" << minDiversity << ".txt";
    std::ofstream file(filename.str().c_str(), std::ofstream::out | std::ofstream::trunc);
    
    file << "GRAPH GENERATION:\n";
    file << " number of states: " << n_states << "\n";
    file << " maximum edge length: " << max_edge_length << "\n";
    file << " P(edge between u,v | d(u,v) < maximum edge length): " << p_connected << "\n\n";
    
    file << "PROBLEM SPECIFICATION:\n";
    file << " find " << n_paths << " paths subject to the constraint that\n";
    file << " all pairs of paths have Levenshtein edit distance at least " << minDiversity << "\n\n\n";
    
    std::vector<PathSetStats> statsExhaustive;
    std::vector<PathSetStats> statsSpace;
    std::vector<PathSetStats> statsGraph;
    
    std::cout << "Running " << n_trials << " trials for each algorithm:\n";
    for (std::size_t i = 0; i < n_trials; i++)
    {
        Graph *g = generateGraph(si, n_states, max_edge_length, p_connected);
        Vertex start = boost::vertex(0, *g);
        Vertex goal = boost::vertex(1, *g);
        
        std::cout << "Exhaustive:\n";
        double runtime = std::clock();
        std::vector<Path> paths = exhaustiveShortestPaths(n_paths, *g, start, goal, minDiversity);
        runtime = std::clock()-runtime;
        statsExhaustive.push_back(PathSetStats(paths, runtime/CLOCKS_PER_SEC));
        
        std::cout << "Space:\n";
        runtime = std::clock();
        std::vector<Path> paths2 = stochasticAvoidancePaths(NBH_STATESPACE, n_paths, start, goal, *g, radius_factor, samples_per_path, minDiversity);
        runtime = std::clock()-runtime;
        statsSpace.push_back(PathSetStats(paths2, runtime/CLOCKS_PER_SEC));
        
        std::cout << "Graph:\n";
        runtime = std::clock();
        std::vector<Path> paths3 = stochasticAvoidancePaths(NBH_GRAPHDISTANCE, n_paths, start, goal, *g, radius_factor, samples_per_path, minDiversity);
        runtime = std::clock()-runtime;
        statsGraph.push_back(PathSetStats(paths3, runtime/CLOCKS_PER_SEC));
        
        std::cout << " completed trial " << i+1 << "/" << n_trials << "\n\n";
        delete g;
    }
    
    file << "All results are computed with respect to the data from the exhaustive search with greedy filtering algorithm.\n\n";
    
    file << "ALGORITHM: Stochastic avoidance sampling using space metric\n";
    file << " (neighborhood radius: " << radius_factor << " * path length)\n";
    file << " (samples per path: " << samples_per_path << ")\n";
    AlgorithmSummary(statsSpace, statsExhaustive).print(file);
    
    file << "ALGORITHM: Stochastic avoidance sampling using graph distance\n";
    file << " (neighborhood radius: " << radius_factor << " * path length)\n";
    file << " (samples per path: " << samples_per_path << ")\n";
    AlgorithmSummary(statsGraph, statsExhaustive).print(file);
    
    file.close();
    
    return 0;
}

Graph *generateGraph(ompl::base::SpaceInformationPtr si, std::size_t n_states, double max_edge_length, double p_connected)
{
    Graph *g = new Graph(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add n_states random states and connect them randomly
    for (std::size_t i = 0; i < n_states; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g->addVertex(state);
    }
    for (std::size_t i = 0; i < n_states; i++)
    {
        for (std::size_t j = 0; j < n_states; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, *g, boost::vertex(i, *g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, *g, boost::vertex(j, *g)).state;
            if (si->getStateSpace()->distance(iState, jState) <= max_edge_length)
            {
                if (float(std::rand())/RAND_MAX < p_connected)
                    g->addEdge(boost::vertex(i, *g), boost::vertex(j, *g));
            }
        }
    }
    
    return g;
}


