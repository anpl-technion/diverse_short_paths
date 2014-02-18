/*
 * Results.cpp
 */

#include "Results.h"

#include "Path.h"

Results::Results (const TestData *const testData, std::vector<Path> &pathSet)
  : data(testData)
{
    for (std::size_t i = 0; i < pathSet.size(); i++)
    {
        paths.push_back(&pathSet[i]);
    }
}

void Results::collate (const Results *, const Results *)
{
}

const TestData *Results::getData () const
{
    return data;
}
