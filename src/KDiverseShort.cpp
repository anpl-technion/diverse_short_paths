/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

// Constructors, destructors

KDiverseShort::KDiverseShort (const TestData *data)
  : too_long(false), i(0), c(0), testData(data)
{
    // Set up path storage and nearest neighbors
    Path::setDistanceFunction(testData->getPathDistanceFunction());
    pathArray = new Path[testData->getK()];
    pathNN = new ompl::NearestNeighborsGNAT<Path>();
    pathNN->setDistanceFunction(&Path::distance);
}

KDiverseShort::~KDiverseShort ()
{
    delete [] pathArray;
    delete pathNN;
}

// Public methods

bool KDiverseShort::considerPath(const Path &path)
{
    // Update user on progress during long runs
    if (++c % 10000 == 0)
        std::cout << "Success rate: " << i << "/" << c << "\n";
    
    // Reject path if it is too long
    if (path.getLength() > testData->getMaxLength())
    {
        too_long = true;
        return false;
    }
    
    // Reject path if it is too close to others
    if (i > 0)
    {
        const Path &nearest = pathNN->nearest(path);
        const double d = (pathNN->getDistanceFunction())(path, nearest);
        if (d < testData->getMinDistance())
            return false;
    }
    
    // Path meets criteria
    pathArray[i++] = path;
    pathNN->add(pathArray[i-1]);
    return true;
}

const Results *KDiverseShort::timedRun ()
{
    // Time the execution
    clock_t start = clock();
    std::string desc = run();
    clock_t end = clock();
    double seconds = ((double)(end-start))/CLOCKS_PER_SEC;
    
    // Label the results
    std::stringstream fullDescription;
    fullDescription << desc;
    const double d = testData->getMinDistance();
    if (d == std::numeric_limits<double>::epsilon())
        fullDescription << ", no filtering";
    else
        fullDescription << ", filtering for " << d << " units of distance";
    return new Results(fullDescription.str(), testData, pathArray, i, seconds);
}

// Protected methods

bool KDiverseShort::tooLong () const
{
    return too_long;
}

bool KDiverseShort::needMore () const
{
    return i < testData->getK();
}
