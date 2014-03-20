/*
 * Main.cpp
 */


#include "pch.h"

#include "Eppstein.h"
#include "Neighborhood.h"
#include "Results.h"
#include "TestData.h"
#include "Voss.h"

/*
 * Run my algorithm on the data.
 * Print results.
 */
void runEppsteinTests (const TestData *data, double &Te, double &De)
{
    double time;
    
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *res = epp->timedRun(time);
    res->saveSet();
    
    // Put results into a nice format
    Te = time;
    De = res->diversity();
    res->print(time);
    delete epp;
    delete res;
}

/*
 * Run my algorithm on the data.
 * Print results.
 */
void runVossTests (const TestData *data, double radiusFactor,
  std::ostream &T, std::ostream &P, std::ostream &D)
{
    double time;
    
    // Run my algorithm on the data
    Voss *voss = new Voss(data, radiusFactor);
    const Results *res = voss->timedRun(time);
    
    // Put results into nice tables
    T << time;
    P << res->numPaths();
    D << res->diversity();
    res->print(time);
    delete voss;
    delete res;
}

/*
 * Write out data as Mathematica code.
 */
void mathematicate (const char *plotName, std::string X,  std::string T,
  std::string P, std::string D, double Te, double De)
{
    std::stringstream ss_Te, ss_De;
    ss_Te << Te;
    ss_De << De;
    // Insert data into plot template
    std::ifstream in("plotTemplate.m.in");
    std::string format((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::size_t length = format.length() + std::strlen(plotName) + X.length() + T.length()
      + P.length() + D.length() + ss_Te.str().length() + ss_De.str().length() + 1;
    char *buf = new char[length];
    std::sprintf(buf, format.c_str(), X.c_str(), T.c_str(), P.c_str(), D.c_str(),
                 ss_Te.str().c_str(), ss_De.str().c_str(), plotName);
    std::cout << buf;
    delete buf;
}

/*
 * Run tests on k diverse short path algorithms.
 */
int main (int argc, char *argv[])
{
    srand(1000);
    
    // Parse command line args
    if (argc != 6)
    {
        std::cerr << "Usage: diverse <graph> -l|-d <maxLength|minDistance> <plotName> <runs>\n";
        std::cerr << argc;
        return -1;
    }
    int arg = 1;
    const char *graphFile = argv[arg++];
    double maxPathLength = std::numeric_limits<double>::infinity();
    double minPathPairwiseDistance = 1e-12;
    if (std::strcmp("-l", argv[arg]) == 0)
        maxPathLength = std::atof(argv[arg+1]);
    else if (std::strcmp("-d", argv[arg]) == 0)
        minPathPairwiseDistance = std::atof(argv[arg+1]);
    arg += 2;
    const char *plotName = argv[arg++];
    const size_t runs = std::atoi(argv[arg++]);
    
    // Build graph to test on
    TestData data(graphFile, 10, maxPathLength, minPathPairwiseDistance);
    
    // Eppstein
    double Te, De;
    runEppsteinTests(&data, Te, De);
    
    // Voss
    std::stringstream X, T, P, D;
    X << "{";
    T << "{";
    P << "{";
    D << "{";
    for (size_t run = 0; run < runs; run++)
    {
        if (run != 0)
        {
            T << ",";
            P << ",";
            D << ",";
        }
        
        // Parameter sweep on the radiusFactor
        T << "{";
        P << "{";
        D << "{";
        for (double rf = 0.0025; rf <= 0.070001; rf += 0.0025)
        {
            if (run == 0)
            {
                if (rf != 0.0025)
                    X << ",";
                X << rf;
            }
            if (rf != 0.0025)
            {
                T << ",";
                P << ",";
                D << ",";
            }
            runVossTests(&data, rf, T, P, D);
        }
        T << "}";
        P << "}";
        D << "}";
    }
    X << "}";
    T << "}";
    P << "}";
    D << "}";
    mathematicate(plotName, X.str(), T.str(), P.str(), D.str(), Te, De);
    
    return 0;
}
