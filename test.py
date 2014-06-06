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

pool = None

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
    Generate a datapoint averaged over many runs for Plot 1 from the given radius and distance measures.
    """

    acc = 0
    runs = float(RUNS)
    for _ in xrange(RUNS):
        algorithm = "r:" + d1 + ":" + d2 + ":" + str(r)
        try:
            datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/grid2.graphml", str(PATHS), algorithm], universal_newlines=True))
        except subprocess.CalledProcessError as e:
            raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)
        if datapoint[0] < PATHS:
            runs -= 1
        else:
            acc += datapoint[3]
    if runs == 0:
        return float("inf")
    return acc/runs

def plot1():
    """
    Compute and plot data for Plot 1.
    """

    global pool
    X = numpy.arange(0.01, 0.5, 0.01)
    Y = [[], [], [], []]
    i = 0
    for d1 in PATH_DISTANCE_MEASURES:
        for d2 in NEIGHBORHOOD_RADIUS_MEASURES:
            data = pool.map_async(plot1F, map(lambda r: (d1, d2, r), X)).get(99999999)
            Y[i] = data
            i += 1

    matplotlib.pyplot.plot(X, Y[0], "r--", X, Y[1], "ys", X, Y[2], "g^", X, Y[3], "bo")
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    matplotlib.pyplot.title("Comparison of Distance Measures")
    matplotlib.pyplot.savefig("plot1.png")

    return


def plot2F((algorithm, g)):
    """
    Generate a single-run datapoint for Plot 2 from the given graph and algorithm.
    """

    try:
        datapoint = extract_datapoint(subprocess.check_output([EXE, "resources/" + g  + ".graphml", str(PATHS), algorithm], universal_newlines=True))
    except subprocess.CalledProcessError as e:
        raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)
    return (datapoint[3], datapoint[4])

def plot2():
    """
    Compute and plot data for Plot 2.
    """

    global pool
    Emin = []
    Emean = []
    for g in GRAPHS:
        data = pool.map_async(plot2F, map(lambda _: ("e:f", g), xrange(RUNS))).get(99999999)
        mins, means = zip(*data)
        Emin.append(sum(mins)/float(RUNS))
        Emean.append(sum(means)/float(RUNS))

    Rmin = []
    Rmean = []
    for g in GRAPHS:
        data = pool.map_async(plot2F, map(lambda _: ("r:f:c:" + str(RADII[g]), g), xrange(RUNS))).get(99999999)
        mins, means = zip(*data)
        Rmin.append(sum(mins)/float(RUNS))
        Rmean.append(sum(means)/float(RUNS))

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

    return
    

def plot3F((algorithm, d)):
    """
    Generate a datapoint averaged over many runs for Plot 3 from the given distance filter and algorithm.
    """
    
    try:
        data = map(lambda _: extract_datapoint(subprocess.check_output(
            [EXE, "resources/grid2.graphml", str(PATHS), algorithm, "-d", str(d)], universal_newlines=True))[5], xrange(RUNS))
    except subprocess.CalledProcessError as e:
        raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)
    return sum(data)/float(RUNS)

def plot3():
    """
    Compute and plot data for Plot 3.
    """

    global pool
    X = numpy.arange(0, 3.5, 0.1)
    E = pool.map_async(plot3F, map(lambda d: ("e:f", d), X)).get(99999999)
    R = pool.map_async(plot3F, map(lambda d: ("r:f:c:" + str(RADII["grid2"]), d), X)).get(99999999)

    matplotlib.pyplot.subplots()
    matplotlib.pyplot.plot(X, E, "ro", X, R, "b^")
    matplotlib.pyplot.xlabel("Minimum Distance")
    matplotlib.pyplot.ylabel("Time (s)")
    matplotlib.pyplot.title("Speed Comparison of Algorithms")
    matplotlib.pyplot.savefig("plot3.png")

    return


def main():
    """
    Run "diverse" to generate data and plot it.
    """
    
    # Setup parallelization
    global pool
    pool = multiprocessing.Pool(int(multiprocessing.cpu_count()*1.2))
    
    # Plots
    print("Generating plot 1")
    plot1()
    print("Generating plot 2")
    plot2()
    print("Generating plot 3")
    plot3()

    return


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print("Testing failed.")
	print(e)
