/*
 * Results.h
 */

#ifndef __RESULTS_H
#define __RESULTS_H

#include "pch.h"

class Path;
class TestData;

/*
 * Representation of experiment results for an algorithm.
 */
class Results
{
private:
    const TestData *const data;
    std::vector<Path *> paths;
    
public:
    
    // Constructor
    Results (const TestData *const testData, std::vector<Path> &pathSet);
    
    // Combine sets of results into pretty format
    static void collate (const Results *r1, const Results *r2);
    
    // Get test data
    const TestData *getData () const;
};

#endif
