#!/usr/bin/env python3

### Script to generate matplotlib plots

import matplotlib.pyplot
import numpy
import subprocess

def extract_datapoint(program_output):
    """
    Extract the 6 numbers from the program's output string.
    """
    
    program_output = program_output.split()
    ix = program_output.index("Found") + 1
    pathsFound = float(program_output[ix])
    ix = program_output.index("length:", ix) + 1
    shortest = float(program_output[ix])
    ix = program_output.index("length:", ix) + 1
    longest = float(program_output[ix])
    ix = program_output.index("neighbor:", ix) + 1
    minDistance = float(program_output[ix])
    ix = program_output.index("neighbor:", ix) + 1
    meanDistance = float(program_output[ix])
    ix = program_output.index("in", ix) + 1
    time = float(program_output[ix])
    
    return (pathsFound, shortest, longest, minDistance, meanDistance, time)

"""
Plots:

1) Parameter sweep of radius on grid2 showing Levenshtein, Frechet, graph distance, cspace distance
"""

PATH_DISTANCE_MEASURES = ["levenshtein", "frechet"]
NEIGHBORHOOD_RADIUS_MEASURES = ["graph", "cspace"]

def main():
    """
    Run "diverse" to generate data and plot it.
    """
    
    try:
        X = numpy.arange(0.01, 0.5, 0.01)
        Y = [[], [], [], []]
        i = 0
        for d1 in PATH_DISTANCE_MEASURES:
            for d2 in NEIGHBORHOOD_RADIUS_MEASURES:
                print("Running parameter sweep using " + d1 + "," + d2)
                for r in X:
                    print(r)
                    algorithm = "r:" + d1 + ":" + d2 + ":" + str(r)
                    datapoint = extract_datapoint(
                        subprocess.check_output(["build/bin/diverse", "resources/grid2.graphml", "10", algorithm], universal_newlines=True)
                    )
                    Y[i].append(datapoint[3])
                i += 1
    except subprocess.CalledProcessError:
        print("Testing failed.")
    
    
    matplotlib.pyplot.plot(X, Y[0], "r--", Y[1], "ys", Y[2], "g^", Y[3], "bo")
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    matplotlib.pyplot.savefig("plot1.png")

if __name__ == "__main__":
    main()

