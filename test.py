#!/usr/bin/env python

### Script to generate matplotlib plots

import matplotlib
# Allow matplotlib to generate images without a display
matplotlib.use('Agg')

import matplotlib.pyplot
import multiprocessing
import numpy
import subprocess

DEBUG = False
EXE = "build/bin/diverse"
RUNS = 2 if DEBUG else 100
PATHS = 10
PATH_DISTANCE_MEASURES = ["levenshtein", "frechet"]
NEIGHBORHOOD_RADIUS_MEASURES = ["graph", "cspace"]
GRAPHS = ["cubicles1", "cubicles2", "cubicles3"]
pool = None

# These numbers aren't magic. For each graph, I pushed the radius as high as it could go while the number
#  of paths returned was still on average at least 80% of the number of paths requested.
RADII = {"cubicles1": 0.002, "cubicles2": 0.013, "cubicles3": 0.012}

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


def find_optimal_radius(graph, distMeasures, runs, tol, _min = 1e-12, _max = 1.0):
    """
    Search for the optimal radius to use on a graph w.r.t the heuristic h. Not very exact because of large
    statistical variation, but useful for ballparking a good radius.
    """

    # Want min and mean distance to be large, with preference to mean; severe penalty for returning fewer paths
    h = lambda (pathsFound, shortest, longest, minDistance, meanDistance, time): \
        (0.8*meanDistance + 0.2*minDistance) * pow(float(pathsFound)/PATHS, 3) if pathsFound > 1 else 0

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
        return (float("inf"), 0)
    return (acc/runs, float(datapoint[0])/PATHS)

def plot1():
    """
    Compute and plot data for Plot 1.
    """

    global pool
    X = numpy.arange(0.01, 0.5, 0.1 if DEBUG else 0.01)
    Y = [[], [], [], []]
    A = [[], [], [], []]
    i = 0
    for d1 in PATH_DISTANCE_MEASURES:
        for d2 in NEIGHBORHOOD_RADIUS_MEASURES:
            data = pool.map_async(plot1F, map(lambda r: (d1, d2, r), X)).get(99999999)
            Y[i], A[i] = zip(*data)
            i += 1


    matplotlib.pyplot.clf()
    for i in xrange(len(X)):
        matplotlib.pyplot.plot(X[:-i], Y[0][:-i], "w:v", markeredgecolor='w')
        matplotlib.pyplot.plot(X[:-i], Y[0][:-i], "b:v", alpha=A[0][-i])
        matplotlib.pyplot.plot(X[:-i], Y[1][:-i], "w--D", markeredgecolor='w')
        matplotlib.pyplot.plot(X[:-i], Y[1][:-i], "r--D", alpha=A[1][-i])
        matplotlib.pyplot.plot(X[:-i], Y[2][:-i], "w-.o", markeredgecolor='w')
        matplotlib.pyplot.plot(X[:-i], Y[2][:-i], "g-.o", alpha=A[2][-i])
        matplotlib.pyplot.plot(X[:-i], Y[3][:-i], "w-^", markeredgecolor='w')
        matplotlib.pyplot.plot(X[:-i], Y[3][:-i], "c-^", alpha=A[3][-i])
    matplotlib.pyplot.xlabel("Radius Factor")
    matplotlib.pyplot.ylabel("Diversity")
    l1 = matplotlib.lines.Line2D([0,1], [0,1], linestyle=":", marker='v', color='b')
    l2 = matplotlib.lines.Line2D([0,1], [0,1], linestyle="--", marker='D', color='r')
    l3 = matplotlib.lines.Line2D([0,1], [0,1], linestyle="-.", marker='o', color='g')
    l4 = matplotlib.lines.Line2D([0,1], [0,1], linestyle="-", marker='^', color='c')
    matplotlib.pyplot.legend((l2, l1, l4, l3), ('Levenshtein, C-Space Distance',
        'Levenshtein, Graph Distance', 'Frechet, C-Space Distance', 'Frechet, Graph Distance'),
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
        radius = RADII[g]
        data = pool.map_async(plot2F, map(lambda _: ("r:f:c:" + str(radius), g), xrange(RUNS))).get(99999999)
        mins, means, paths = zip(*data)
        mins = reduce(lambda l, e: (l + [e] if e != float('inf') else l), mins, [])
        means = reduce(lambda l, e: (l + [e] if e != float('inf') else l), means, [])
        print(str(100*float(sum(paths))/(PATHS*RUNS)) + "% completion for graph " + g)
        # We would like an average success rate of 75% or better
        if sum(paths) < 0.75*PATHS*RUNS:
            print("Consider decreasing radius for " + g)
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
    matplotlib.pyplot.ylim([0,150])
    matplotlib.pyplot.legend((sty3[0], sty1[0], sty4[0], sty2[0]),
              ('Eppstein, min', 'Eppstein, mean', 'Random Avoidance, min', 'Random Avoidance, mean'), 'upper right')
    matplotlib.pyplot.title("Diversity of Path Set")
    matplotlib.pyplot.savefig("plot2.png")

    return
    

def plot3F((g, algorithm, d)):
    """
    Generate a datapoint averaged over many runs for Plot 3 from the given distance filter and algorithm.
    """
    
    try:
        return extract_datapoint(subprocess.check_output(
            [EXE, "resources/"+g+".graphml", str(PATHS), algorithm, "-d", str(d)], universal_newlines=True))[5]
    except subprocess.CalledProcessError as e:
        raise Exception("subprocess.CalledProcessError: exit status " + str(e.returncode) + "\nCalled: " + ' '.join(e.cmd) + "\nReturned: " + e.output)

def plot3(graph, radius):
    """
    Compute and plot data for Plot 3.
    """

    global pool
    X = numpy.arange(1e-12, 5.1, 1 if DEBUG else 0.1)
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
        data = pool.map_async(plot3F, map(lambda _: ("r:f:c:0.1", d), xrange(RUNS))).get(99999999)
        R.append(numpy.mean(data))
        Rerr.append(numpy.std(data))

    matplotlib.pyplot.clf()
    R = numpy.array(R)
    Rerr = numpy.array(Rerr)
    l1, l2 = matplotlib.pyplot.plot(X, E, "-", X, R, "--")
    matplotlib.pyplot.plot(X, R+Rerr, "c-.", X, R-Rerr, "c-.")
    matplotlib.pyplot.xlabel("Minimum Distance Required between Paths")
    matplotlib.pyplot.ylabel("Time (s)")
    matplotlib.pyplot.xlim([0,5])
    matplotlib.pyplot.ylim([0,25])
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
    #print("cubicles1 optimal radius: " + str(find_optimal_radius("cubicles1", "f:c", 50, 0.00001, 1e-12, 0.1)))
    #print("cubicles2 optimal radius: " + str(find_optimal_radius("cubicles2", "f:c", 50, 0.00001, 1e-12, 0.1)))
    #print("cubicles3 optimal radius: " + str(find_optimal_radius("cubicles3", "f:c", 50, 0.00001, 1e-12, 0.1)))
    
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
