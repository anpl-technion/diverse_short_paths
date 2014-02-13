/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "TestData.h"

KDiverseShort::KDiverseShort (const TestData *data)
: too_long(false), testData(data)
{
}

KDiverseShort::~KDiverseShort ()
{
}

bool KDiverseShort::tooLong () const
{
    return too_long;
}

bool KDiverseShort::needMore () const
{
    return pathSet.size() < testData->getK();
}

bool KDiverseShort::considerPath(const Path &path)
{
    if (path.getLength() > testData->getMaxLength())
    {
        too_long = true;
        return false;
    }
    
    double nearest_distance = testData->getMinDistance();
    for (std::size_t i = 0; i < pathSet.size(); i++)
    {
        double distance = testData->getGraph().levenshteinDistance(pathSet[i], path);
        if (distance < nearest_distance)
            distance = nearest_distance;
    }
    if (nearest_distance < testData->getMinDistance())
        return false;
        
    pathSet.push_back(path);
    return true;
}
