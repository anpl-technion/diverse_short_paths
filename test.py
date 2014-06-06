#!/usr/bin/env python

### Script to generate matplotlib plots

import matplotlib
# Allow matplotlib to generate images without a display
matplotlib.use('Agg')

import matplotlib.pyplot
import multiprocessing
import numpy
import subprocess

EXE = "build/bin/diverse"
RUNS = 10
PATHS = 10
PATH_DISTANCE_MEASURES = ["levenshtein", "frechet"]
NEIGHBORHOOD_RADIUS_MEASURES = ["graph", "cspace"]
GRAPHS = ["grid1", "grid2"]
RADII = {"grid1": 0.2, "grid2": 0.2} #, "cubicles1": 0.01}

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
3) Parameter sweep of minimum distance filtering, comparing Eppstein with random avoidance on grid2 based on speed
"""

def plot1F((d1, d2, r)):
    """
    Generate a datapoint for Plot 1 from the given radius and distance measures.
    """

    acc = 0
    runs = float(RUNS)
    for _ in xrange(RUNS):
        algorithm = "r:" + d1 + ":" + d2 + ":" + str(r)
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/grid2.graphml", str(PATHS), algorithm], universal_newlines=True))
        if datapoint[0] < PATHS:
            runs -= 1
        else:
            acc += datapoint[3]
    return acc/runs

def main():
    """
    Run "diverse" to generate data and plot it.
    """
    
    # Setup parallelization
    pool = multiprocessing.Pool()
    
    # Plot 1
    X = numpy.arange(0.01, 0.5, 0.01)
    Y = [[], [], [], []]
    i = 0
    for d1 in PATH_DISTANCE_MEASURES:
        for d2 in NEIGHBORHOOD_RADIUS_MEASURES:
            print("Running parameter sweep using " + d1 + "," + d2)
            data = pool.map(plot1F, map(lambda r: (d1, d2, r), X))
            Y[i] = data
            i += 1
    
    print(X, Y)
    matplotlib.pyplot.plot(X, Y[0], "r--", X, Y[1], "ys", X, Y[2], "g^", X, Y[3], "bo")
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    matplotlib.pyplot.title("Comparison of Distance Measures")
    matplotlib.pyplot.savefig("plot1.png")
    
    # Plot 2
    Emin = []
    Emean = []
    Rmin = []
    Rmean = []
    for g in GRAPHS:
        algorithm1 = "e:f"
        algorithm2 = "r:f:c:" + str(RADII[g])
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/" + g + ".graphml", str(PATHS), algorithm1], universal_newlines=True))
        Emin.append(datapoint[3])
        Emean.append(datapoint[4])
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/" + g + ".graphml", str(PATHS), algorithm2], universal_newlines=True))
        Rmin.append(datapoint[3])
        Rmean.append(datapoint[4])
    
    print(Emin, Emean, Rmin, Rmean)
    fig, ax = matplotlib.pyplot.subplots()
    width = 0.35
    inner = 0.7
    ticks = numpy.arange(len(GRAPHS))
    sty1 = ax.bar(ticks, Emean, width, color='r')
    sty2 = ax.bar(width+ticks, Rmean, width, color='b')
    sty3 = ax.bar(((1-inner)/2)*width+ticks, Emin, inner*width, color='y')
    sty4 = ax.bar((1+(1-inner)/2)*width+ticks, Rmin, inner*width, color='g')
    ax.set_xticks(ticks+width)
    ax.set_xticklabels(GRAPHS)
    ax.set_xlabel("Graph")
    ax.set_ylabel("Diversity")
    ax.legend((sty3[0], sty1[0], sty4[0], sty2[0]),
              ('Eppstein, min', 'Eppstein, mean', 'Random Avoidance, min', 'Random Avoidance, mean'))
    ax.set_title("Distance Between Paths")
    matplotlib.pyplot.savefig("plot2.png")
    
    # Plot 3
    X = numpy.arange(0, 3.5, 0.1)
    E = []
    R = []
    algorithm1 = "e:f"
    algorithm2 = "r:f:c:" + str(RADII["grid2"])
    for d in X:
        print(d)
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/grid2.graphml", str(PATHS), algorithm1, "-d", str(d)], universal_newlines=True))
        E.append(datapoint[5])
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/grid2.graphml", str(PATHS), algorithm2, "-d", str(d)], universal_newlines=True))
        R.append(datapoint[5])
    
    print(E, R)
    matplotlib.pyplot.subplots()
    matplotlib.pyplot.plot(X, E, "ro", X, R, "b^")
    matplotlib.pyplot.xlabel("Minimum Distance")
    matplotlib.pyplot.ylabel("Time (s)")
    matplotlib.pyplot.title("Speed Comparison of Algorithms")
    matplotlib.pyplot.savefig("plot3.png")
    
if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print("Testing failed.")
	print(e)
