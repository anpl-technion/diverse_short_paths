
/* Author: Caleb Voss */

#ifndef DRAW_H
#define DRAW_H

#include "Graph.h"
#include "Path.h"

void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename);

#endif
