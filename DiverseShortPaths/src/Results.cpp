/*
 * Results.cpp
 */

#include "Results.h"

Results::Results ()
{ }

void Results::addPath (const std::vector<Vertex> path)
{
    paths.push_back(path);
}

void Results::collate (const Results *, const Results *)
{
}
