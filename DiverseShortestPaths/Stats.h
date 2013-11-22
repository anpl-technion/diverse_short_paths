
/* Author: Caleb Voss */

#ifndef STATS_H
#define STATS_H

#include <fstream>

#include "Path.h"

class StatSummary
{
protected:
    
    double numPaths;
    double runtime;
    double maxPairwiseDistance;
    double meanPairwiseDistance;
    double meanNNDistance;
    double maxPathLength;
    double meanPathLength;
    
public:
    
    StatSummary (void);
    
    double getNumPaths (void) const;
    
    double getRuntime (void) const;
    
    double getMaxPairwiseDistance (void) const;
    
    double getMeanPairwiseDistance (void) const;
    
    double getMeanNNDistance (void) const;
    
    double getMaxPathLength (void) const;
    
    double getMeanPathLength (void) const;
};

class PathSetStats : public StatSummary
{
    void computePairwiseDistances (const std::vector<Path> &paths);

    void computeNNDistances (const std::vector<Path> &paths);

    void computePathLengths (const std::vector<Path> &paths);
        
public:
    
    PathSetStats (const std::vector<Path> &paths, double runningTime);
};

class AlgorithmSummary : public StatSummary
{
    std::size_t successful;
    std::size_t bad_graph;
    double failurePercentage;
    
public:
    
    AlgorithmSummary (const std::vector<PathSetStats> &stats, const std::vector<PathSetStats> &withRespectTo);
    
    void print (std::ofstream &file) const;
};

#endif
