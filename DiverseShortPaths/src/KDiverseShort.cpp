/*
 * KDiverseShort.cpp
 */

#include "KDiverseShort.h"

#include "Graph.h"
#include "Path.h"
#include "Results.h"
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
            nearest_distance = distance;
    }
    if (nearest_distance < testData->getMinDistance())
        return false;
        
    pathSet.push_back(path);
    path.print();
    return true;
}

const Results *KDiverseShort::getResults (const char *alg_name)
{
    std::stringstream ss;
    ss << "algorithm_" << alg_name << ":" << testData->getParameter();
    return new Results(ss.str(), testData, pathSet);
}
