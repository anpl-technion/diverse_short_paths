/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

// Constructors, destructors

KDiverseShort::KDiverseShort (const TestData *data, PathDistanceMeasure *pDist)
: too_long(false), i(0), c(0), testData(data)
{
    // Set up path storage and nearest neighbors
    Path::setDistanceFunction(boost::bind(&PathDistanceMeasure::distance, pDist, _1, _2));
    pDistName = pDist->getName();
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

const Results *KDiverseShort::timedRun ()
{
    // Time the execution
    clock_t start = clock();
    run();
    clock_t end = clock();
    double seconds = ((double)(end-start))/CLOCKS_PER_SEC;
    
    // Label the results
    const double d = testData->getMinDistance();
    if (d <= std::numeric_limits<double>::epsilon())
    {
        const double l = testData->getMaxLength();
        if (l == std::numeric_limits<double>::infinity())
            description << ", no filtering";
        else
            description << ", filtering for maximum length " << l;
    }
    else
        description << ", filtering for pairwise distance " << d;
    description << " (" << pDistName << ")";
    const Results *res = new Results(description.str(), testData, pathArray, i, seconds);
    return res;
}

void KDiverseShort::clear ()
{
    // Prepare for future runs
    too_long = false;
    i = 0;
    c = 0;
    pathNN->clear();
    description.str("");
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
        if (Path::distance(path, nearest) < testData->getMinDistance())
            return false;
    }
    
    // Path meets criteria
    pathArray[i++] = path;
    pathNN->add(pathArray[i-1]);
    return true;
}
