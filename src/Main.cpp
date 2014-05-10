/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Frechet.h"
#include "Levenshtein.h"
#include "Neighborhood.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/*
 * Write out data as Mathematica code.
 */
void mathematicate (std::string X,  std::string T,
  std::string P, std::string D, std::string L, double Te, double De)
{
    std::stringstream ss_Te, ss_De;
    ss_Te << Te;
    ss_De << De;
    // Insert data into plot template
    std::ifstream in("plotTemplate.m.in");
    std::string format((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::size_t length = format.length() + X.length() + T.length()
      + P.length() + D.length() + L.length() + ss_Te.str().length() + ss_De.str().length() + 1;
    char *buf = new char[length];
    std::sprintf(buf, format.c_str(), X.c_str(), T.c_str(), P.c_str(), D.c_str(),
                 L.c_str(), ss_Te.str().c_str(), ss_De.str().c_str());
    std::cout << buf;
    delete buf;
}

/**
 * Print a usage message and exit.
 * @warning Terminates the program.
 */
void usage ()
{
    std::cerr << "Usage: diverse <graphml> <paths> <algorithm> { -u | -l <maxLength> | -d <minDistance> } [-s]\n";
    std::cerr << "    <graphml> a file in *.graphml format\n";
    std::cerr << "    <paths> is the number of paths to find\n";
    std::cerr << "    <algorithm> is of the form <name>:<pathDistance>[:<neighborhoodDistance>:<neighborhoodRadius>]\n";
    std::cerr << "        <name> is 'eppstein' or 'randomavoidance'\n";
    std::cerr << "        <pathDistance> is 'levenshtein' or 'frechet'\n";
    std::cerr << "        <neighborhoodDistance> is 'cspace' or 'graph'; it is required by 'randomavoidance'\n";
    std::cerr << "        <neighborhoodRadius> is a float in (0,1); it is required by 'randomavoidance'\n";
    std::cerr << "      * algorithm specifications may be abbreviated using e,r,l,f,c,g\n";
    std::cerr << "    <maxLength>,<minDistance> are floats specifying constraints on returned paths\n";
    std::cerr << "    -s flag enables saving the path set in a file called paths.txt\n\n";
    std::cerr << "Example: diverse mygraph.graphml e:l -d 5.7 3\n";
    std::exit(-1);
}
/*
 * Run tests on k diverse short path algorithms.
 */
int main (int argc, char *argv[])
{
    srand(1000);
    
    // Parse command line args
    int maxargs = 7;
    if (argc != maxargs-1 && argc != maxargs)
        usage();
    std::size_t arg = 1;
    const char *graphFile = argv[arg++];
    int paths = std::atoi(argv[arg++]);
    if (paths < 0)
    {
        std::cerr << "Number of paths should not be negative!\n";
        std::exit(-1);
    }
    
    // Algorithm specifications
    char *parser = argv[arg++];
    char *s;
    const char *algorithm(strsep(&parser, ":"));
    const char *pathDistance((s = strsep(&parser, ":")) ? s : "?");
    const char *neighborhoodDistance((s = strsep(&parser, ":")) ? s : "?");
    const char *neighborhoodRadius((s = strsep(&parser, ":")) ? s : "-1");
    const double radiusFactor = std::atof(neighborhoodRadius);
    
    double maxPathLength = std::numeric_limits<double>::infinity();
    double minPathPairwiseDistance = std::numeric_limits<double>::epsilon();
    if (std::strcmp("-l", argv[arg]) == 0)
    {
        if (argc != maxargs)
            usage();
        maxPathLength = std::atof(argv[++arg]);
    }
    else if (std::strcmp("-d", argv[arg]) == 0)
    {
        if (argc != maxargs)
            usage();
        minPathPairwiseDistance = std::atof(argv[++arg]);
    }
    else if (std::strcmp("-u", argv[arg]) == 0)
    {
        if (argc != maxargs-1)
            usage();
    }
    else
        usage();
    const bool save = (std::strcmp("-s", argv[++arg]) == 0);
    
    // Build graph to test on
    TestData data(graphFile, paths, maxPathLength, minPathPairwiseDistance);
    
    // Choose distance function
    PathDistanceMeasure *distanceMeasure = nullptr;
    if (strcasecmp(pathDistance, "f") == 0 || strcasecmp(pathDistance, "frechet") == 0)
        distanceMeasure = new Frechet();
    else if (strcasecmp(pathDistance, "l") == 0 || strcasecmp(pathDistance, "levenshtein") == 0)
        distanceMeasure = new Levenshtein();
    else
    {
        std::cout << "Error: Unknown distance measure '" << pathDistance << "'\n";
        std::exit(-1);
    }
    
    // Choose neighborhood avoidance method (if any)
    Neighborhood::AvoidMethod avoidMethod = Neighborhood::UNKNOWN;
    if (strcasecmp(neighborhoodDistance, "c") == 0 || strcasecmp(neighborhoodDistance, "cspace") == 0)
        avoidMethod = Neighborhood::CSPACE;
    else if (strcasecmp(neighborhoodDistance, "g") == 0 || strcasecmp(neighborhoodDistance, "graph") == 0)
        avoidMethod = Neighborhood::GRAPH;
    
    // Choose algorithm
    KDiverseShort *kDiverseShort = nullptr;
    if (strcasecmp(algorithm, "e") == 0 || strcasecmp(algorithm, "eppstein") == 0)
        kDiverseShort = new Eppstein(&data, distanceMeasure);
    else if (strcasecmp(algorithm, "r") == 0 || strcasecmp(algorithm, "randomavoidance") == 0)
        kDiverseShort = new Voss(&data, distanceMeasure, radiusFactor, avoidMethod);
    else
    {
        std::cout << "Error: Unknown algorithm '" << algorithm << "'\n";
        std::exit(-1);
    }
    
    
    std::cout << "\n";
    const Results *res = kDiverseShort->timedRun();
    res->print();
    if (save)
        res->saveSet();
    delete res;
    delete kDiverseShort;
    delete distanceMeasure;
    
    return 0;
}
