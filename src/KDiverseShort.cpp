/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "Results.h"
#include "TestData.h"

KDiverseShort::KDiverseShort (const TestData *data)
  : too_long(false), c(0), pathNN(new ompl::NearestNeighborsGNAT<Path>()), testData(data)
{
    pathArray = new Path[testData->getK()];
    i = 0;
    pathNN->setDistanceFunction(&Path::distance);
}

KDiverseShort::~KDiverseShort ()
{
    delete [] pathArray;
    delete pathNN;
}

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
    if (++c % 10000 == 0)
        std::cout << "Success rate: " << i << "/" << c << "\n";
    
    if (path.getLength() > testData->getMaxLength())
    {
        too_long = true;
        return false;
    }
    
    if (i > 0)
    {
        const Path &nearest = pathNN->nearest(path);
        const double d = (pathNN->getDistanceFunction())(path, nearest);
        if (d < testData->getMinDistance())
            return false;
    }
    
    pathArray[i++] = path;
    pathNN->add(pathArray[i-1]);
    return true;
}

const Results *KDiverseShort::getResults (const char *alg_name)
{
    return new Results(alg_name, testData, pathArray, i);
}

const Results *KDiverseShort::timedRun (double &seconds)
{
    // Time the execution
    clock_t start = clock();
    const Results *res = run();
    clock_t end = clock();
    
    seconds = ((double)(end-start))/CLOCKS_PER_SEC;
    return res;
}

