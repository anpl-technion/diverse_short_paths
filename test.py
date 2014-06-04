#!/usr/bin/env python3

### Script to generate matplotlib plots

import matplotlib.pyplot
import numpy
import subprocess

EXE = "build/bin/diverse"
PATH_DISTANCE_MEASURES = ["levenshtein", "frechet"]
NEIGHBORHOOD_RADIUS_MEASURES = ["graph", "cspace"]
GRAPHS = ["grid1", "grid2"]
RADII = {"grid1": 0.3, "grid2": 0.3} #, "cubicles1": 0.01}

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
2) Comparison of Eppstein with random avoidance on several graphs using Frechet and cspace distance
    showing both min and mean distance superimposed
3) Parameter sweep of minimum distance filtering, comparing Eppstein with random avoidance on grid2 
"""

def main():
    """
    Run "diverse" to generate data and plot it.
    """
    
    # Plot 1
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
                        subprocess.check_output(
                            [EXE, "resources/grid2.graphml", "10", algorithm],
                            universal_newlines=True))
                    if datapoint[0] < 10:
                        Y[i].append(float("inf"))
                    else:
                        Y[i].append(datapoint[3])
                i += 1
    except subprocess.CalledProcessError:
        print("Testing failed.")
    
    print(X, Y)
    matplotlib.pyplot.plot(X, Y[0], "r--", X, Y[1], "ys", X, Y[2], "g^", X, Y[3], "bo")
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    matplotlib.pyplot.savefig("plot1.png")
    
    # Plot 2
    try:
        E = []
        R = []
        for g in GRAPHS:
            algorithm1 = "e:f"
            algorithm2 = "r:f:c:" + str(RADII[g])
            datapoint = extract_datapoint(
                subprocess.check_output(
                    [EXE, "resources/" + g + ".graphml", "10", algorithm1],
                    universal_newlines=True))
            E.append(datapoint[3])
            datapoint = extract_datapoint(
                subprocess.check_output(
                    [EXE, "resources/" + g + ".graphml", "10", algorithm2],
                    universal_newlines=True))
            R.append(datapoint[3])
    except subprocess.CalledProcessError:
        print("Testing failed.")
    
    print(E, R)
    fig, ax = matplotlib.pyplot.subplots()
    ax.bar(numpy.arange(len(GRAPHS)), E, 0.35)
    ax.bar(0.35+numpy.arange(len(GRAPHS)), R, 0.35)
    matplotlib.pyplot.savefig("plot2.png")

if __name__ == "__main__":
    main()

