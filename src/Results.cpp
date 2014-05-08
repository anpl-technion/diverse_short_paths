/*
 * Results.cpp
 */

#include "Results.h"

#include "Path.h"
#include "TestData.h"

// Constructors, destructors

Results::Results (const std::string &desc, const TestData *testData,
                  const Path *pathSet, std::size_t nPaths, double runtime)
  : description(desc), data(testData), paths(pathSet), n(nPaths), t(runtime)
{
}

// Public methods

void Results::saveSet () const
{
    // Open a file and save each path in turn
    std::ofstream fout(description + ".txt");
    for (std::size_t i = 0; i < n; i++)
    {
        paths[i].saveOMPLFormat(fout);
    }
}

void Results::print () const
{
    std::cout << "Description: " << description << "\n";
    std::cout << " Found " << n << " of " << data->getK() << " requested paths.\n";
    const double shortest = findShortestLength();
    std::cout << "\tshortest path length: " << shortest << "\n";
    const double longest = findLongestLength();
    std::cout << "\tlongest path length:  " << longest << " (" << longest/shortest << " times as long)\n";
    std::cout << "\tmin distance to nearest neighbor:  " << minNearestPathDistance() << "\n";
    std::cout << "\tmean distance to nearest neighbor: " << meanNearestPathDistance() << "\n";
    std::cout << " Completed in " << t << " seconds\n\n\n";
}

double Results::getTime () const
{
    return t;
}

double Results::findShortestLength () const
{
    // Assume it's the first one
    if (n > 0)
        return paths[0].getLength();
    else
        return std::numeric_limits<double>::quiet_NaN();
}

double Results::findLongestLength () const
{
    double longest = 0;
    for (std::size_t i = 0; i < n; i++)
    {
        if (paths[i].getLength() > longest)
            longest = paths[i].getLength();
    }
    return longest;
}

double Results::minNearestPathDistance () const
{
    double min = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; i++)
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
    for (std::size_t i = 0; i < n; i++)
    {
        sum += nearestPathDistance(i);
    }
    return sum/n;
}

double Results::nearestPathDistance (const std::size_t which) const
{
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; i++)
    {
        // Don't check against itself
        if (i == which)
            continue;
        const double distance = Path::distance(paths[i], paths[which]);
        if (distance < nearest)
            nearest = distance;
    }
    return nearest;
}

std::size_t Results::numPaths () const
{
    return n;
}
