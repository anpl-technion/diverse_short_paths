
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// If NO_DRAW is defined, no output images are produced and cairo library is not required
#ifndef NO_DRAW
#include <cairo/cairo.h>
#endif

#include "Graph.h"
#include "Path.h"
#include "ActualShortest.h"

// Magic number
#define NUM_AVOIDS_PER_PATH     5

/** \brief Get a random state from the path (not equal to the start or end of the path)
 * 
 * @tparam N        Neighborhood class to use
 * 
 * @param path      path to sample from
 * @param g         graph this path exists in
 * 
 * @return          pointer to a state on the path
 */
template <class N>
typename N::center_type sampleFromPath (const std::list<Vertex> &path, const Graph &g);
template <>
ompl::base::State *sampleFromPath <StateSpaceNeighborhood> (const std::list<Vertex> &path, const Graph &g)
{
    std::list<Vertex>::const_iterator vi = path.begin();
    for (std::size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return boost::get(boost::vertex_prop, g, *vi).state;
}
template <>
Vertex sampleFromPath <GraphDistanceNeighborhood> (const std::list<Vertex> &path, const Graph &g)
{
    std::list<Vertex>::const_iterator vi = path.begin();
    for (std::size_t i = std::rand() % path.size(); i > 0; i--)
        vi++;
    return *vi;
}


/** \brief Find a number of diverse, short paths in a graph, by deviating from former short paths through
 *         the use of an increasing set of neighborhoods in the state space that should be avoided
 * 
 * @tparam N                Neighborhood class to use
 * 
 * @param numPaths          number of paths to try to find
 * @param start             start vertex for the path
 * @param end               end vertex for the path
 * @param g                 graph to search in
 * @param radius_factor     neighborhood radius is radius_factor * pathLength
 * @param minDiversity      paths are not retained if their levenshtein edit distance from any already retained path is less than this
 * 
 * @return                  set of at most numPaths distinct paths from start to end
 */
template <class N>
std::vector<Path> findDiverseShortestPaths (std::size_t numPaths, Vertex start, Vertex end, const Graph &g, double radius_factor, unsigned int minDiversity)
{
    // Holds the set of paths we've found
    std::vector<Path> resultPaths;
    if (numPaths == 0)
        return resultPaths;
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<N> > resultAvoids;
    // The next path to analyze
    std::size_t frontier = 0;
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<N> alreadyAvoiding = std::vector<N>();
    Path referencePath = g.getShortestPathWithAvoidance<N>(start, end, alreadyAvoiding);
    if (referencePath.getPath().empty())
        return resultPaths;
    
    resultPaths.push_back(referencePath);
    resultAvoids.push_back(alreadyAvoiding);
    
    // Work through the queue until we have enough
    while (frontier < resultPaths.size() && resultPaths.size() < numPaths)
    {
        referencePath = resultPaths[frontier];
        alreadyAvoiding = resultAvoids[frontier];
        frontier++;
        
        // Make a pre-defined number of attempts at imposing a new neighborhood to avoid on the graph
        // Neighborhood's radius is some pre-defined portion of the referencePath's length
        double radius = radius_factor * referencePath.getLength();
        for (int i = 0; i < NUM_AVOIDS_PER_PATH && resultPaths.size() < numPaths; i++)
        {
            std::vector<N> avoid = alreadyAvoiding;
            avoid.push_back(N(sampleFromPath<N>(referencePath.getPath(), g), radius));
            // Get the shortest path under these constraints
            Path path = g.getShortestPathWithAvoidance<N>(start, end, avoid);
            if (path.getPath().empty())
                continue;
            
            // Don't store it if it's not diverse enough
            bool tooSimilar = false;
            BOOST_FOREACH(Path p, resultPaths)
            {
                if (levenshtein(path.getPath(), p.getPath()) < minDiversity)
                {
                    tooSimilar = true;
                    break;
                }
            }
            if (!tooSimilar)
            {
                resultPaths.push_back(path);
                resultAvoids.push_back(avoid);
            }
        }
    }
    
    return resultPaths;
}

#ifndef NO_DRAW
void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename);
#endif
void printStats (Graph &g, std::vector<Path> &paths);

int main (int argc, char **argv)
{
    if (argc != 5)
    {
        std::cout << "expected 4 args: <n_states> <n_paths> <radius_factor> <minDiversity>\n";
        return 0;
    }
    size_t n_states;
    std::istringstream(argv[1]) >> n_states;
    size_t n_paths;
    std::istringstream(argv[2]) >> n_paths;
    double radius_factor;
    std::istringstream(argv[3]) >> radius_factor;
    unsigned int minDiversity;
    std::istringstream(argv[4]) >> minDiversity;
    
    // Tweakable params for graph generation
    const double max_edge_length = 4.0/std::sqrt(n_states/100.0);
    const double p_connected = 0.25;
    
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add n_states random states and connect them randomly
    for (size_t i = 0; i < n_states; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    for (size_t i = 0; i < n_states; i++)
    {
        for (size_t j = 0; j < n_states; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, g, boost::vertex(i, g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, g, boost::vertex(j, g)).state;
            if (space->distance(iState, jState) <= max_edge_length)
            {
                if (float(std::rand())/RAND_MAX < p_connected)
                    g.addEdge(boost::vertex(i, g), boost::vertex(j, g));
            }
        }
    }
    
    std::cout << "Finding at most 10 diverse short paths from node 0 to node 8\n\n";
    Vertex start = boost::vertex(0, g);
    Vertex goal = boost::vertex(8, g);
    std::vector<Path> paths = findDiverseShortestPaths<StateSpaceNeighborhood>(n_paths, start, goal, g, radius_factor, minDiversity);
    std::cout << "Completed space distance version\n";
    std::vector<Path> paths2 = findDiverseShortestPaths<GraphDistanceNeighborhood>(n_paths, start, goal, g, radius_factor, minDiversity);
    std::cout << "Completed graph distance version\n";
    std::vector<Path> paths3 = actualShortestPaths(n_paths, g, start, goal, minDiversity);
    std::cout << "Completed actual shortest paths\n";
    std::cout << "Got " << paths.size() << ", " << paths2.size() << ", " << paths3.size() << " paths\n";
    
#ifndef NO_DRAW
    draw(g, start, goal, paths, "output_space.png");
    draw(g, start, goal, paths2, "output_graph.png");
    draw(g, start, goal, paths3, "output_actual.png");
#endif
    
    if (paths.size() > 1)
    {
        std::cout << "\nSeparation of paths with state space neighborhoods:\n";
        printStats(g, paths);
    }
    if (paths2.size() > 1)
    {
        std::cout << "\nSeparation of paths with graph distance neighborhoods:\n";
        printStats(g, paths2);
    }
    if (paths3.size() > 1)
    {
        std::cout << "\nSeparation of the actual shortest paths:\n";
        printStats(g, paths3);
    }
    
    return 0;
}

#ifndef NO_DRAW
void draw (Graph &g, Vertex start, Vertex goal, std::vector<Path> &paths, const char *filename)
{
    cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1024, 1024);
    cairo_t *cr = cairo_create(surface);
    cairo_set_line_width(cr, 1);
    
    // Draw each edge as a black line
    BOOST_FOREACH(Vertex v, boost::vertices(g))
    {
        ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
        double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        BOOST_FOREACH(Vertex u, boost::adjacent_vertices(v, g))
        {
            state = boost::get(boost::vertex_prop, g, u).state;
            double *ucoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            
            cairo_move_to(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50);
            cairo_line_to(cr, 12+(ucoords[0]+10)*50, 12+(ucoords[1]+10)*50);
            cairo_stroke(cr);
        }
    }
    
    // Draw each path edge as line with thickness and color according to how soon it was found
    int line_width = 2*paths.size()+1;
    float red = 1;
    float green = 0.8;
    float blue = 0;
    cairo_set_line_cap  (cr, CAIRO_LINE_CAP_ROUND);
    BOOST_FOREACH(Path path, paths)
    {
        cairo_set_line_width(cr, line_width);
        cairo_set_source_rgba(cr, red, green, blue, 1);
        Vertex u = path.getPath().front();
        BOOST_FOREACH(Vertex v, path.getPath())
        {
            if (u != v)
            {
                ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
                double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                state = boost::get(boost::vertex_prop, g, u).state;
                double *ucoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                
                cairo_move_to(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50);
                cairo_line_to(cr, 12+(ucoords[0]+10)*50, 12+(ucoords[1]+10)*50);
                cairo_stroke(cr);
            }
            u = v;
        }
        line_width -= 2;
        red -= 1.0/(paths.size()+1);
        blue += 1.0/(paths.size()+1);
    }
    
    // Draw each vertex as a black dot; except start and goal should be pink
    BOOST_FOREACH(Vertex v, boost::vertices(g))
    {
        ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
        double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        if (v == start || v == goal)
            cairo_set_source_rgba(cr, 1, 0, 1, 1);
            
        cairo_arc(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50, 3, 0, 2*M_PI);
        cairo_fill(cr);
        cairo_set_source_rgba(cr, 0, 0, 0, 1);
    }
    
    cairo_surface_write_to_png(surface, filename);
}
#endif

void printStats (Graph &g, std::vector<Path> &paths)
{
    // Compute Levenshtein edit distance between every pair as a diversity measure
    unsigned int min = std::numeric_limits<unsigned int>::max();
    unsigned int max = 0;
    unsigned int total = 0;
    unsigned int count = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1.getPath() == path2.getPath())
                continue;
            unsigned int distance = levenshtein(path1.getPath(), path2.getPath());
            if (distance < min)
                min = distance;
            if (distance > max)
                max = distance;
            total += distance;
            count++;
        }
    }
    
    // Report min, max, and mean
    std::cout << "DISTANCE BETWEEN PATHS: min: " << min << ", max: " << max << ", mean: " << double(total)/count << "\n";
    
    // Compute Levenshtein edit distance between every path and it's nearest neighbor
    min = std::numeric_limits<unsigned int>::max();
    total = 0;
    BOOST_FOREACH(Path path1, paths)
    {
        Path nearest(path1);
        unsigned int distance = 0;
        BOOST_FOREACH(Path path2, paths)
        {
            if (path1.getPath() == path2.getPath())
                continue;
            distance = levenshtein(path1.getPath(), path2.getPath());
            if (distance < min)
                min = distance;
        }
        total += distance;
    }
    
    // Report min, max, and mean
    std::cout << "DISTANCE TO NEAREST NEIGHBOR: mean: " << double(total)/paths.size() << "\n";
    
    // Compute length of every path
    double lmin = std::numeric_limits<double>::max();
    double lmax = 0;
    double ltotal = 0;
    BOOST_FOREACH(Path path, paths)
    {
        double distance = pathLength(path.getPath(), g);
        if (distance < lmin)
            lmin = distance;
        if (distance > max)
            lmax = distance;
        ltotal += distance;
    }
    
    // Report min, max, and mean
    std::cout << "LENGTH OF PATHS: min: " << lmin << ", max: " << lmax << ", mean: " << double(ltotal)/paths.size() << "\n";
}

unsigned int levenshtein (const std::list<Vertex> &path1, const std::list<Vertex> &path2)
{
    const size_t rowLength = path1.size()+1;
    const size_t colLength = path2.size()+1;
    unsigned int *const distances = new unsigned int[rowLength*colLength];
    
    for (size_t j = 0; j < rowLength; j++)
        distances[j] = j;
    for (size_t i = 1; i < colLength; i++)
        distances[i*rowLength] = i;
    
    std::list<Vertex>::const_iterator iIt = path1.begin();
    for (size_t i = 1; i < colLength; i++, iIt++)
    {
        std::list<Vertex>::const_iterator jIt = path2.begin();
        for (size_t j = 1; j < rowLength; j++, jIt++)
        {
            const unsigned int del = distances[(i-1)*rowLength+j] + 1;
            const unsigned int ins = distances[i*rowLength+j-1] + 1;
            unsigned int match = distances[(i-1)*rowLength+j-1];
            if (*iIt != *jIt)
                match++;
            distances[i*rowLength+j] = std::min(std::min(del, ins), match);
        }
    }
    
    const unsigned int d = distances[rowLength*colLength-1];
    delete [] distances;
    return d;
}

// Compare with via-point method
