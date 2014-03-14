/*
 * Results.cpp
 */

#include "Results.h"

#include "Path.h"
#include "TestData.h"

Results::Results (std::string name, const TestData *const testData, std::vector<Path> &pathSet)
  : description(name), data(testData)
{
    paths.reserve(pathSet.size());
    for (std::size_t i = 0; i < pathSet.size(); i++)
    {
        paths.push_back(&pathSet[i]);
    }
}

void Results::print (double runtime) const
{
    std::clog << "Description: " << description << "\n";
    std::clog << " Found " << paths.size() << " of " << data->getK() << " requested paths.\n";
    const double shortest = findShortestLength();
    std::clog << "\tshortest path length is " << shortest << "\n";
    const double longest = findLongestLength();
    std::clog << "\tlongest path length is " << longest << " (" << longest/shortest << " times as long)\n";
    std::clog << "\tmean distance to nearest neighbor is " << meanNearestPathDistance() << "\n";
    std::clog << " Completed in " << runtime << " seconds\n";
    
    std::clog << "\n\n" << std::flush;
}

void Results::saveSet () const
{
    std::ofstream fout(description + ".txt");
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        paths[i]->saveOMPLFormat(fout);
    }
}

double Results::findShortestLength () const
{
    // Assume it's the first one
    if (paths.size() > 0)
        return paths[0]->getLength();
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double Results::findLongestLength () const
{
    double longest = 0;
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        if (paths[i]->getLength() > longest)
            longest = paths[i]->getLength();
    }
    return longest;
}

double Results::diversity () const
{
//     return meanNearestPathDistance();
    return minNearestPathDistance();
}

double Results::minNearestPathDistance () const
{
    double min = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        double d = nearestPathDistance(i);
        if (d < min)
            min = d;
    }
    return min;
}

double Results::meanNearestPathDistance () const
{
    double sum = 0;
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        sum += nearestPathDistance(i);
    }
    return sum/paths.size();
}

double Results::nearestPathDistance (const std::size_t which) const
{
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < paths.size(); i++)
    {
        if (i == which)
            continue;
        const double distance = Path::distance(*paths[i], *paths[which]);
        if (distance < nearest)
            nearest = distance;
    }
    return nearest;
}

const TestData *Results::getData () const
{
    return data;
}

std::size_t Results::numPaths () const
{
    return paths.size();
}

const Path *Results::getPath (std::size_t i) const
{
    return paths[i];
}
