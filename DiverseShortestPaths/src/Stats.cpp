
/* Author: Caleb Voss */

#include "Stats.h"

#include <limits>

#include <boost/foreach.hpp>

StatSummary::StatSummary (void)
 : numPaths(0), runtime(0), maxPairwiseDistance(0), meanPairwiseDistance(0), meanNNDistance(0), maxPathLength(0), meanPathLength(0)
{
}

double StatSummary::getNumPaths (void) const
{
    return numPaths;
}

double StatSummary::getRuntime (void) const
{
    return runtime;
}

double StatSummary::getMaxPairwiseDistance (void) const
{
    return maxPairwiseDistance;
}

double StatSummary::getMeanPairwiseDistance (void) const
{
    return meanPairwiseDistance;
}

double StatSummary::getMeanNNDistance (void) const
{
    return meanNNDistance;
}

double StatSummary::getMaxPathLength (void) const
{
    return maxPathLength;
}

double StatSummary::getMeanPathLength (void) const
{
    return meanPathLength;
}

PathSetStats::PathSetStats (const std::vector<Path> &paths, double runningTime)
{
    runtime = runningTime;
    numPaths = paths.size();
    
    if (paths.size() <= 1)
    {
        maxPairwiseDistance = meanPairwiseDistance = meanNNDistance = NAN;
    }
    else
    {
        computePairwiseDistances(paths);
        computeNNDistances(paths);
    }
    
    computePathLengths(paths);
}

void PathSetStats::computePairwiseDistances (const std::vector<Path> &paths)
{
    maxPairwiseDistance = 0;
    unsigned int total = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1 == path2)
                continue;
            unsigned int distance = Path::levenshtein(path1, path2);
            if (distance > maxPairwiseDistance)
                maxPairwiseDistance = distance;
            total += distance;
        }
    }
    
    meanPairwiseDistance = double(total)/(paths.size() * (paths.size()-1));
}

void PathSetStats::computeNNDistances (const std::vector<Path> &paths)
{
    unsigned int total = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        unsigned int min = std::numeric_limits<unsigned int>::max();
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1 == path2)
                continue;
            unsigned int distance = Path::levenshtein(path1, path2);
            if (distance < min)
                min = distance;
        }
        total += min;
    }
    
    meanNNDistance = double(total)/paths.size();
}

void PathSetStats::computePathLengths (const std::vector<Path> &paths)
{
    maxPathLength = 0;
    double total = 0;
    BOOST_FOREACH(Path path, paths)
    {
        double distance = path.getLength();
        if (distance > maxPathLength)
            maxPathLength = distance;
        total += distance;
    }
    
    meanPathLength = total/paths.size();
}

AlgorithmSummary::AlgorithmSummary (const std::vector<PathSetStats> &stats, const std::vector<PathSetStats> &withRespectTo)
{
    if (stats.size() != withRespectTo.size())
        throw "Vectors must contain the same number of trials!\n";
    
    std::vector<PathSetStats>::const_iterator respectiveTrial = withRespectTo.begin();
    successful = 0;
    bad_graph = 0;
    BOOST_FOREACH(PathSetStats stat, stats)
    {
        if (respectiveTrial->getNumPaths() <= 1)
        {
            // Throw the result out without counting failure because not even the exhaustive algorithm could do well
            respectiveTrial++;
            bad_graph++;
            continue;
        }
        if (stat.getNumPaths() <= 1)
        {
            // Throw this result out as a failure
            respectiveTrial++;
            continue;
        }
        successful++;
        numPaths += stat.getNumPaths() / respectiveTrial->getNumPaths();
        runtime += stat.getRuntime() / respectiveTrial->getRuntime();
        maxPairwiseDistance += stat.getMaxPairwiseDistance() / respectiveTrial->getMaxPairwiseDistance();
        meanPairwiseDistance += stat.getMeanPairwiseDistance() / respectiveTrial->getMeanPairwiseDistance();
        meanNNDistance += stat.getMeanNNDistance() / respectiveTrial->getMeanNNDistance();
        maxPathLength += stat.getMaxPathLength() / respectiveTrial->getMaxPathLength();
        meanPathLength += stat.getMeanPathLength() / respectiveTrial->getMeanPathLength();
        
        respectiveTrial++;
    }
    
    numPaths /= successful;
    runtime /= successful;
    maxPairwiseDistance /= successful;
    meanPairwiseDistance /= successful;
    meanNNDistance /= successful;
    maxPathLength /= successful;
    meanPathLength /= successful;
    
    failurePercentage = 100 * (1.0-double(successful+bad_graph)/stats.size());
}

void AlgorithmSummary::print (std::ofstream &file) const
{
    file << " failure rate: " << failurePercentage << "%\n";
    file << "Average Results (over " << successful << " successful trials):\n";
    file << " running time: " << getRuntime() << "\n";
    file << " paths found: " << getNumPaths() << "\n";
    file << " pairwise edit distance:\n";
    file << "    max:  " << getMaxPairwiseDistance() << "\n";
    file << "    mean: " << getMeanPairwiseDistance() << "\n";
    file << " nearest neighbor edit distance:\n";
    file << "    mean: " << getMeanNNDistance() << "\n";
    file << " path length:\n";
    file << "    max:  " << getMaxPathLength() << "\n";
    file << "    mean: " << getMeanPathLength() << "\n";
    file << "\n\n";
}
