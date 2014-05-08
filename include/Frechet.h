/*
 * Frechet.h
 */

#ifndef __FRECHET_H
#define __FRECHET_H

#include "pch.h"

/**
 * Container for the discrete Frechet distance algorithm.
 */
class Frechet
{
public:
    
    /**
     * Compute the distance between two paths according to the discrete Frechet algorithm.
     * @param path1 first path
     * @param path2 second path
     * @return distance between \a path1 and \a path2
     * @warning Does not check if paths are in the same graph.
     */
    static double distance (const Path &path1, const Path &path2);
};

#endif
