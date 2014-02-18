/*
 * Results.cpp
 */

#include "Results.h"

#include "Path.h"

Results::Results (const char *alg_name, const TestData *const testData, std::vector<Path> &pathSet)
  : algorithm(alg_name), data(testData)
{
    paths.reserve(pathSet.size());
    for (std::size_t i = 0; i < pathSet.size(); i++)
    {
        paths.push_back(&pathSet[i]);
    }
}

void Results::print (std::size_t i) const
{
    std::cout << "DATA SET " << i << "\n";
    std::cout << "Algorithm: " << algorithm << "\n";
    
    std::cout << "\n\n";
}

void Results::collate (const Results *r1, const Results *r2)
{
    r1->print(1);
    r2->print(2);
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
