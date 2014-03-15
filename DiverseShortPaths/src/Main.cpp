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
void runEppsteinTests (const TestData *data)
{
    double time;
    
    // Run Eppstein's algorithm on the data
    Eppstein *epp = new Eppstein(data);
    const Results *eppstein_r = epp->timedRun(time);
    eppstein_r->saveSet();
    
    // Put results into a nice format
    eppstein_r->print(time);
    delete epp;
    delete eppstein_r;
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
    const Results *voss_r = voss->timedRun(time);
    
    // Put results into nice tables
    T << time << ",";
    P << voss_r->numPaths() << ",";
    D << voss_r->diversity() << ",";
    voss_r->print(time);
    delete voss;
    delete voss_r;
}

/*
 * Write out data as Mathematica code.
 */
void mathematicate (const char *plotName, std::string X,  std::string T,
  std::string P, std::string D)
{
    // Insert data into plot template
    std::ifstream in("plotTemplate.m.in");
    std::string format((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::size_t length = format.length() + std::strlen(plotName) + X.length() + T.length()
      + P.length() + D.length() + 1;
    char *buf = new char[length];
    std::sprintf(buf, format.c_str(), X.c_str(), T.c_str(), P.c_str(), D.c_str(), plotName);
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
    if (argc != 4)
    {
        std::cerr << "Usage: diverse <graph> <maxLength> <plotName>\n";
        std::cerr << argc;
        return -1;
    }
    const char *graphFile = argv[1];
    const double maxPathLength = std::atof(argv[2]);
    const double minPathPairwiseDistance = 0;//std::atof(argv[3]);
    const char *plotName = argv[3];
    
    // Build graph to test on
    TestData data(graphFile, 10, maxPathLength, minPathPairwiseDistance);
    
//     // Eppstein
//     // Fix an upper limit for path length and test it
//     data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
//     runEppsteinTests(&data);
//     
//     // Fix a lower limit for distance between pairs of paths and test it
//     data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
//     runEppsteinTests(&data);
    
    // Voss
    std::stringstream X, T, P, D;
    X << "{";
    T << "{";
    P << "{";
    D << "{";
    for (size_t run = 0; run < 10; run++)
    {
        T << "{";
        P << "{";
        D << "{";
        // Parameter sweep on the radiusFactor
        for (double rf = 0.0025; rf <= 0.050001; rf += 0.0025)
        {
            // Fix an upper limit for path length and test it
            data.setMode(TestData::Mode::FIX_MAX_PATH_LENGTH);
            if (run == 0)
                X << rf << ",";
            runVossTests(&data, rf, T, P, D);
            
//             // Fix a lower limit for distance between pairs of paths and test it
//             data.setMode(TestData::Mode::FIX_MIN_PATH_DISTANCE);
//             runVossTests(&data, rf);
        }
        T << "\b},";
        P << "\b},";
        D << "\b},";
    }
    X << "\b}";
    T << "\b}";
    P << "\b}";
    D << "\b}";
    mathematicate(plotName, X.str(), T.str(), P.str(), D.str());
    
    return 0;
}
