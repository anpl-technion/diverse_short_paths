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

int usage ()
{
    std::cerr << "Usage: diverse <graphml> <algorithm> { -u | -l <maxLength> | -d <minDistance> } <runs>\n";
    std::cerr << "    <graphml> a file in *.graphml format\n";
    std::cerr << "    <algorithm> is of the form <name>:<pathDistance>[:<neighborhoodDistance>:<neighborhoodRadius>]\n";
    std::cerr << "        <name> is 'kshortest' or 'randomavoidance'\n";
    std::cerr << "        <pathDistance> is 'levenshtein' or 'frechet'\n";
    std::cerr << "        <neighborhoodDistance> is 'cspace' or 'graph'; it is required by 'randomavoidance'\n";
    std::cerr << "        <neighborhoodRadius> is a float; it is required by 'randomavoidance'\n";
    std::cerr << "      * algorithm specifications may be abbreviated using k,r,l,f,c,g\n";
    std::cerr << "    <maxLength>,<minDistance> are floats specifying constraints on returned paths\n";
    std::cerr << "    <runs> indicates how many runs to perform\n";
    return -1;
}
/*
 * Run tests on k diverse short path algorithms.
 */
int main (int argc, char *argv[])
{
    srand(1000);
    
    // Parse command line args
    if (argc != 5 && argc != 6)
        return usage();
    int arg = 1;
    const char *graphFile = argv[arg++];
    
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
        if (argc != 6)
            return usage();
        maxPathLength = std::atof(argv[++arg]);
    }
    else if (std::strcmp("-d", argv[arg]) == 0)
    {
        if (argc != 6)
            return usage();
        minPathPairwiseDistance = std::atof(argv[++arg]);
    }
    else
    {
        if (argc != 5)
            return usage();
    }
    //const size_t runs = std::atoi(argv[++arg]);
    
    // Build graph to test on
    TestData data(graphFile, 10, maxPathLength, minPathPairwiseDistance);
    
    // Choose distance function
    Path::DistanceFunction distanceFunction = nullptr;
    if (strcasecmp(pathDistance, "f") == 0 || strcasecmp(pathDistance, "frechet") == 0)
        distanceFunction = &Frechet::distance;
    else if (strcasecmp(pathDistance, "l") == 0 || strcasecmp(pathDistance, "levenshtein") == 0)
        distanceFunction = &Levenshtein::distance;
    
    // Choose neighborhood avoidance method (if any)
    Neighborhood::AvoidMethod avoidMethod = Neighborhood::UNKNOWN;
    if (strcasecmp(neighborhoodDistance, "c") == 0 || strcasecmp(neighborhoodDistance, "cspace") == 0)
        avoidMethod = Neighborhood::CSPACE;
    else if (strcasecmp(neighborhoodDistance, "g") == 0 || strcasecmp(neighborhoodDistance, "graph") == 0)
        avoidMethod = Neighborhood::GRAPH;
    
    // Choose algorithm
    KDiverseShort *kDiverseShort = nullptr;
    if (strcasecmp(algorithm, "e") == 0 || strcasecmp(algorithm, "eppstein") == 0)
        kDiverseShort = new Eppstein(&data, distanceFunction);
    else if (strcasecmp(algorithm, "r") == 0 || strcasecmp(algorithm, "randomavoidance") == 0)
        kDiverseShort = new Voss(&data, distanceFunction, radiusFactor, avoidMethod);
    
    std::cout << "\n";
    const Results *res = kDiverseShort->timedRun();
    res->print();
    delete res;
    delete kDiverseShort;
    
    /*
    // Eppstein
    Eppstein epp(&data);
    const Results *res = epp.timedRun();
    
    // Put results into a nice format
    double Te, De;
    Te = res->getTime();
    De = res->minNearestPathDistance();
    res->print();
    res->saveSet();
    delete res;
    
    // Voss
    std::stringstream X, T, P, D, L;
    X << "{";
    T << "{";
    P << "{";
    D << "{";
    L << "{";
    for (size_t run = 0; run < runs; run++)
    {
        std::cout << "Sweep " << run << "\n";
        if (run != 0)
        {
            T << ",";
            P << ",";
            D << ",";
            L << ",";
        }
        
        // Parameter sweep on the radiusFactor
        T << "{";
        P << "{";
        D << "{";
        L << "{";
        //for (double rf = 0.0025; rf <= 0.160001; rf += 0.0025)
        //for (double rf = 0.0600; rf <= 0.160001; rf += 0.0025)
        double rf = 0.13;
        {
            if (run == 0)
            {
                X << ",";
                X << rf;
            }
            T << ",";
            P << ",";
            D << ",";
            L << ",";

            Voss voss(&data, rf, Neighborhood::CSPACE);
            const Results *res = voss.timedRun();
            
            // Put results into nice tables
            T << res->getTime();
            P << res->numPaths();
            D << res->minNearestPathDistance();
            L << res->findLongestLength();
            res->print();
            res->saveSet();
            delete res;
        }
        T << "}";
        P << "}";
        D << "}";
        L << "}";
    }
    X << "}";
    T << "}";
    P << "}";
    D << "}";
    L << "}";
    //mathematicate(X.str(), T.str(), P.str(), D.str(), L.str(), Te, De);
    */
    return 0;
}
