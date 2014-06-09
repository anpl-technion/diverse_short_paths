#!/usr/bin/env python

### Script to generate matplotlib plots

import matplotlib
# Allow matplotlib to generate images without a display
matplotlib.use('Agg')

import matplotlib.pyplot
import multiprocessing
import numpy
import subprocess

DEBUG = True
EXE = "build/bin/diverse"
RUNS = 2 if DEBUG else 50
PATHS = 10
PATH_DISTANCE_MEASURES = ["levenshtein", "frechet"]
NEIGHBORHOOD_RADIUS_MEASURES = ["graph", "cspace"]
GRAPHS = ["cubicles1", "cubicles2", "cubicles3"]

# These were chosen based on results from the find_optimal_radius() function.
RADII = {"grid1": 0.10, "grid2": 0.10, "cubicles1": 0.000128, "cubicles2": 0.00002, "cubicles3": 0.0258}

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


def find_optimal_radius(graph, distMeasures, runs, tol, _min = 1e-12, _max = 10):
    """
    Search for the optimal radius to use on a graph w.r.t the heuristic h.
    """

    # Want min and mean distance to be large, with preference to min; should succeed in finding as many paths as possible
    h = lambda (pathsFound, shortest, longest, minDistance, meanDistance, time): \
        (0.8*minDistance + 0.2*meanDistance) * (float(pathsFound)/PATHS) if pathsFound > 1 else 0

    step = (_max-_min)/10.0
    if _max-_min <= tol:
        return (_min+_max)/2.0
    search = dict.fromkeys(numpy.arange(_min, _max, step))

    for r in search:
        data = pool.map_async(findradiusF, map(lambda _: ("r:" + distMeasures + ":" + str(r), graph), xrange(runs))).get(99999999)
        scores = map(h, data)
        search[r] = sum(scores)/float(len(scores))

    bestr = None
    score = float('-inf')
    for r in search:
        if search[r] >= score:
            bestr = r
            score = search[r]
    
    print("Radius: " + str(bestr) + "; Score: " + str(score))
    return find_optimal_radius(graph, distMeasures, runs, tol, bestr-step if bestr-step > 0 else 1e-12, bestr+step)

def findradiusF((algorithm, g)):

    try:
        return extract_datapoint(subprocess.check_output([EXE, "resources/" + g + ".graphml", str(PATHS), algorithm], universal_newlines=True))
    except subprocess.CalledProcessError as e:
        raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)


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
        if datapoint[0] <= 1:
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
    X = numpy.arange(0.01, 0.5, 0.1 if DEBUG else 0.01)
    Y = [[], [], [], []]
    i = 0
    for d1 in PATH_DISTANCE_MEASURES:
        for d2 in NEIGHBORHOOD_RADIUS_MEASURES:
            data = pool.map_async(plot1F, map(lambda r: (d1, d2, r), X)).get(99999999)
            Y[i] = data
            i += 1

    matplotlib.pyplot.clf()
    l1, l2, l3, l4 = matplotlib.pyplot.plot(X, Y[0], ":x", X, Y[1], "--D", X, Y[2], "-.o", X, Y[3], "-^")
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    matplotlib.pyplot.figlegend((l1, l2, l3, l4), ('Levenshtein, Graph Distance',
        'Levenshtein, C-Space Distance', 'Frechet, Graph Distance', 'Frechet, C-Space Distance'),
        'upper left')
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
    return (datapoint[3], datapoint[4], datapoint[0])

def plot2():
    """
    Compute and plot data for Plot 2.
    """

    global pool
    Emin = []
    Emean = []
    for g in GRAPHS:
        data = plot2F(("e:f", g))
        Emin.append(data[0])
        Emean.append(data[1])

    Rmin = []
    RminErr = []
    Rmean = []
    RmeanErr = []
    for g in GRAPHS:
        data = pool.map_async(plot2F, map(lambda _: ("r:f:c:" + str(RADII[g]), g), xrange(RUNS))).get(99999999)
        mins, means, paths = zip(*data)
        means = reduce(lambda l, e: (l + [e] if e != float('inf') else l), means, [])
        for p in paths:
            if p < 0.7*PATHS:
                print("Need to decrease radius for graph " + g)
        mins = numpy.array(mins)
        means = numpy.array(means)
        Rmin.append(numpy.mean(mins))
        RminErr.append(numpy.std(mins))
        Rmean.append(numpy.mean(means))
        RmeanErr.append(numpy.std(means))

    matplotlib.pyplot.clf()
    width = 0.35
    inner = 0.7
    ticks = numpy.arange(len(GRAPHS))
    sty1 = matplotlib.pyplot.bar(ticks, Emean, width, color='r')
    sty2 = matplotlib.pyplot.bar(width+ticks, Rmean, width, color='b', yerr=RmeanErr, ecolor='k')
    sty3 = matplotlib.pyplot.bar(((1-inner)*0.8)*width+ticks, Emin, inner*width, color='y')
    sty4 = matplotlib.pyplot.bar((1+(1-inner)*0.8)*width+ticks, Rmin, inner*width, color='g', yerr=RminErr, ecolor='k')
    matplotlib.pyplot.xticks(ticks+width, GRAPHS)
    matplotlib.pyplot.xlabel("Graph")
    matplotlib.pyplot.ylabel("Diversity (Frechet)")
    matplotlib.pyplot.ylim([0,130])
    matplotlib.pyplot.legend((sty3[0], sty1[0], sty4[0], sty2[0]),
              ('Eppstein, min', 'Eppstein, mean', 'Random Avoidance, min', 'Random Avoidance, mean'), 'upper left')
    matplotlib.pyplot.title("Diversity of Path Set")
    matplotlib.pyplot.savefig("plot2.png")

    return
    

def plot3F((algorithm, d)):
    """
    Generate a datapoint averaged over many runs for Plot 3 from the given distance filter and algorithm.
    """
    
    try:
        return extract_datapoint(subprocess.check_output(
            [EXE, "resources/grid2.graphml", str(PATHS), algorithm, "-d", str(d)], universal_newlines=True))[5]
    except subprocess.CalledProcessError as e:
        raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)

def plot3():
    """
    Compute and plot data for Plot 3.
    """

    global pool
    X = numpy.arange(1e-12, 5, 2 if DEBUG else 0.1)
    E = []
    Eerr = []
    R = []
    Rerr = []
    for d in X:
        # We will skip Eppstein on large values because it takes way too long
        if d < 4.5:
            data = plot3F(("e:f", d))
            E.append(data)
        else:
            E.append(float('inf'))
        data = pool.map_async(plot3F, map(lambda _: ("r:f:c:" + str(RADII["grid2"]), d), xrange(RUNS))).get(99999999)
        R.append(numpy.mean(data))
        Rerr.append(numpy.std(data))

    matplotlib.pyplot.clf()
    R = numpy.array(R)
    Rerr = numpy.array(Rerr)
    l1, l2 = matplotlib.pyplot.plot(X, E, "-", X, R, "--")
    matplotlib.pyplot.plot(X, R+Rerr, "-.", X, R-Rerr, "-.")
    matplotlib.pyplot.xlabel("Minimum Distance Required between Paths")
    matplotlib.pyplot.ylabel("Time (s)")
    matplotlib.pyplot.ylim([0,15])
    matplotlib.pyplot.legend((l1, l2), ('Eppstein', 'Random Avoidance'), 'upper left')
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

    #print("grid1 optimal radius: " + str(find_optimal_radius("grid1", "f:c", 50, 0.00001)))
    #print("grid2 optimal radius: " + str(find_optimal_radius("grid2", "f:c", 50, 0.00001)))
    #print("cubicles1 optimal radius: " + str(find_optimal_radius("cubicles1", "f:c", 50, 0.00001)))
    #print("cubicles2 optimal radius: " + str(find_optimal_radius("cubicles2", "f:c", 50, 0.00001)))
    #print("cubicles3 optimal radius: " + str(find_optimal_radius("cubicles3", "f:c", 50, 0.00001)))
    
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
