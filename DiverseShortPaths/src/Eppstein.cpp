/*
 * Eppstein.cpp
 */

#include "Eppstein.h"

#include "graehl/kbest.h"

#include "Graph.h"
#include "Path.h"
#include "TestData.h"
#include "Results.h"

Eppstein::Eppstein (const TestData *data)
  : KDiverseShort(data)
{
    graehl_kernel = initGraehl(makeGraehlGraph(data->getGraph()), data->getStart(), data->getEnd());
}

Eppstein::~Eppstein ()
{
    delete graehl_kernel;
}

const Results *Eppstein::run ()
{
    while (!tooLong() && needMore())
    {
        graehl::List<graehl::GraphArc *> *raw = graehl_kernel->getNextPath();
        if (raw == NULL)
            break;
        Path path((Graph *)&testData->getGraph());
        double length = 0;
        graehl::GraphArc *edge;
        while (raw->notEmpty())
        {
            edge = raw->top();
            length += edge->weight;
            path.push_back(edge->source);
            raw->pop();
        }
        path.push_back(edge->dest);
        considerPath(path);
    }
    BOOST_FOREACH(Path path, pathSet)
    {
        path.print();
    }
    std::cout << "\n\n";
    return getResults();
}

std::stringstream *Eppstein::makeGraehlGraph (const Graph &g)
{
    std::stringstream &ss = *new std::stringstream();
    
    // Write the number of vertices
    ss << g.getNumVertices();
    
    // Write each edge with distance
    g.foreachEdge([&] (const Edge e) -> void
    {
        Vertex u, v;
        g.getVertices(e, &u, &v);
        ss << "(" << u << " " << v << " " << g.getEdgeWeight(e) << ")";
    });
    
    return &ss;
}

graehl::Graehl *Eppstein::initGraehl (std::stringstream *input, const std::size_t start, const std::size_t end)
{
    // Graehl was written for ints, so make sure our numbers aren't too big
    assert(start < INT_MAX);
    assert(end < INT_MAX);
    graehl::Graehl *g = new graehl::Graehl(input, (int)start, (int)end);
    delete input;
    return g;
}
