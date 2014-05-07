/*
 * Levenshtein.h
 */

#ifndef __LEVENSHTEIN_H
#define __LEVENSHTEIN_H

#include "pch.h"

class Levenshtein
{
public:
    static double distance (const Path &path1, const Path &path2);
};

#endif
