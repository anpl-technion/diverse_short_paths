/*
 * Levenshtein.h
 */

#ifndef __LEVENSHTEIN_H
#define __LEVENSHTEIN_H

#include "pch.h"

/**
 * Container for the Levenshtein edit distance algorithm.
 */
class Levenshtein
{
public:
    
    /**
     * Compute the distance between two paths according to Levenshtein edit distance.
     * @param path1 first path
     * @param path2 second path
     * @return distance between \a path1 and \a path2
     * @warning Does not check if paths are in the same graph.
     */
    static double distance (const Path &path1, const Path &path2);
};

#endif
