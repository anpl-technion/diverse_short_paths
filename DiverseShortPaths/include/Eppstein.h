/*
 * Eppstein.h
 */

#ifndef __EPPSTEIN_H
#define __EPPSTEIN_H

#include "pch.h"

#include "graehl/kbest.hh"

#include "KDiverseShort.h"

class Graph;
class TestData;

/*
 * Eppstein's algorithm for k shortest paths.
 */
class Eppstein : public KDiverseShort
{
private:
    // Kernel of Graehl code
    graehl::Graehl *graehl_kernel;
    
    static const char *EPP_NAME;
    
public:
    // Construct the algorithm to use the given data
    Eppstein (const TestData *data);
    
    // Destructor
    ~Eppstein ();
    
    // Evaluate the algorithm
    const Results *run ();
    
private:
    // Convert our Graph to the Graehl graph format
    static std::stringstream *makeGraehlGraph (const Graph &g);
    
    // Set up the Graehl code for the graph
    static graehl::Graehl *initGraehl (std::stringstream *input, const std::size_t start, const std::size_t end);
};

#endif
