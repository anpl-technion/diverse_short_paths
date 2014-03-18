/*
 * Path.cpp
 */

#include "Path.h"

#include "Graph.h"

Path::Path (const Path &path)
 :  std::vector<Vertex>(path), g(path.getGraph()), parametrization(path.getPartialEdgeSums())
{
}

Path::Path (std::vector<Vertex> &path, const Graph *graph)
 :  std::vector<Vertex>(path), g(graph)
{
    parametrization.reserve(size());
    parametrization.push_back(0);
    for (std::size_t i = 1; i < size(); i++)
    {
        parametrization.push_back(parametrization[i-1] + g->getEdgeWeight((*this)[i-1], (*this)[i]));
    }
}

Path::Path (const Graph *graph)
 :  std::vector<Vertex>(), g(graph)
{
}

void Path::saveOMPLFormat(std::ostream &out) const
{
    ompl::base::State *state = g->getSpaceInfo()->allocState();
    for (std::size_t i = 1; i < size(); i++)
    {
        for (double interp = 0; interp < 1; interp += 0.2)
        {
            ompl::base::State *s1 = g->getVertexState((*this)[i-1]);
            ompl::base::State *s2 = g->getVertexState((*this)[i]);
            g->getSpaceInfo()->getStateSpace()->interpolate(s1, s2, interp, state);
            std::vector<double> reals;
            g->getSpaceInfo()->getStateSpace()->copyToReals(reals, state);
            for (std::size_t j = 0; j < reals.size(); j++)
            {
                if (j > 0)
                    out << " ";
                out << reals[j];
            }
            out << "\n";
        }
    }
    g->getSpaceInfo()->freeState(state);
}

double Path::getLength () const
{
    return parametrization[size()-1];
}

const Graph *Path::getGraph () const
{
    return g;
}

void Path::print () const
{
    for (std::size_t i = 0; i < size(); i++)
    {
        std::cout << (*this)[i] << " ";
    }
    std::cout << ": " << (empty() ? 0 : getLength()) << "\n";
}

void Path::printWithWeights () const
{
    for (std::size_t i = 0; i < size()-1; i++)
    {
        const double w = g->getEdgeWeight((*this)[i], (*this)[i+1]);
        std::cout << (*this)[i] << " (" << w << ") ";
    }
    std::cout << (*this)[size()-1] << " : " << getLength() << "\n";
}

void Path::push_back (const Vertex &vertex)
{
    std::vector<Vertex>::push_back(vertex);
    if (size() == 1)
        parametrization.push_back(0);
    else
        parametrization.push_back(parametrization[size()-2]
            + g->getEdgeWeight((*this)[size()-2], (*this)[size()-1]));
}

std::vector<double> Path::getPartialEdgeSums () const
{
    return parametrization;
}

ompl::base::State *Path::sampleUniform () const
{
    // Sample between [0,length]
    double par = getLength() * ((double)rand() / (double)RAND_MAX);
    
    // Find vertices to interpolate between
    std::size_t i = 0;
    while (!(parametrization[i] <= par && par <= parametrization[i+1]))
    {
        i++;
    }
    
    par = (par-parametrization[i]) / getLength();
    
    // Interpolate
    ompl::base::State *sample = g->getSpaceInfo()->allocState();
    g->getSpaceInfo()->getStateSpace()->interpolate(g->getVertexState((*this)[i]), g->getVertexState((*this)[i+1]), par, sample);
    return sample;
}

const std::vector<double> &Path::getWeights () const
{
    if (!isWeightCached())
        cacheWeights();
    
    return weights;
}

bool Path::isWeightCached () const
{
    return weights.size() > 0;
}

void Path::cacheWeights () const
{
    Vertex prev = (*this)[0];
    BOOST_FOREACH(Vertex v, *this)
    {
        if (prev == v)
            continue;
        weights.push_back(g->getEdgeWeight(prev, v));
        prev = v;
    }
}

const std::vector<ompl::base::State *> &Path::getStates () const
{
    if (!isStateCached())
        cacheStates();
    
    return states;
}

bool Path::isStateCached () const
{
    return states.size() > 0;
}

void Path::cacheStates () const
{
    BOOST_FOREACH(Vertex v, *this)
    {
        states.push_back(g->getVertexState(v));
    }
}

double Path::distance (const Path &p1, const Path &p2)
{
    return p1.getGraph()->levenshteinDistance(p1, p2);
}