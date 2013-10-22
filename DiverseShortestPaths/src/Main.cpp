
/* Author: Caleb Voss */

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cairo/cairo.h>

#include "Graph.h"

// Magic numbers
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

/** \brief Compute the length of a path by summing the distance between states
 * 
 * @tparam N        Neighborhood class to use
 * 
 * @param path      path to find the length of
 * @param g         graph this path exists in
 * 
 * @return          length of path
 */
template <class N>
double pathLength (const std::list<Vertex> &path, const Graph &g);
template <>
double pathLength <StateSpaceNeighborhood> (const std::list<Vertex> &path, const Graph &g)
{
    double length = 0;
    std::list<Vertex>::const_iterator vi = path.begin();
    vi++;
    while (vi != path.end())
    {
        vi--;
        Vertex u = *vi;
        vi++;
        Vertex v = *vi;
        vi++;
        length += boost::get(boost::edge_weight, g, boost::edge(u, v, g).first);
    }
    return length;
}
template <>
double pathLength <GraphDistanceNeighborhood> (const std::list<Vertex> &path, const Graph &g)
{
    return path.size();
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
 * 
 * @return                  set of at most numPaths distinct paths from start to end
 */
template <class N>
std::vector<std::list<Vertex> > findDiverseShortestPaths (std::size_t numPaths, Vertex start, Vertex end, const Graph &g, double radius_factor)
{
    // Holds the set of paths we've found
    std::vector<std::list<Vertex> > resultPaths;
    if (numPaths == 0)
        return resultPaths;
    // Holds the set of avoided neighborhoods that each path in resultsPaths was made with
    std::vector<std::vector<N> > resultAvoids;
    // The next path to analyze
    std::size_t frontier = 0;
    // The path and its avoided neighborhoods that we will try to diverge from (initially the actual shortest path)
    std::vector<N> alreadyAvoiding = std::vector<N>();
    std::list<Vertex> referencePath = g.getShortestPathWithAvoidance<N>(start, end, alreadyAvoiding);
    if (referencePath.empty())
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
        typename N::radius_type radius = radius_factor * pathLength<N>(referencePath, g);
        for (int i = 0; i < NUM_AVOIDS_PER_PATH && resultPaths.size() < numPaths; i++)
        {
            std::vector<N> avoid = alreadyAvoiding;
            avoid.push_back(N(sampleFromPath<N>(referencePath, g), radius));
            // Get the shortest path under these constraints
            std::list<Vertex> path = g.getShortestPathWithAvoidance<N>(start, end, avoid);
            if (path.empty())
                continue;
            
            // Don't store it if it's a duplicate
            bool alreadyHave = false;
            BOOST_FOREACH(std::list<Vertex> p, resultPaths)
            {
                if (path == p)
                {
                    alreadyHave = true;
                    break;
                }
            }
            if (!alreadyHave)
            {
                resultPaths.push_back(path);
                resultAvoids.push_back(avoid);
            }
        }
    }
    
    return resultPaths;
}

void draw (Graph &g, Vertex start, Vertex goal, std::vector<std::list<Vertex> > paths, const char *filename);

int main (int argc, char **argv)
{
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    Graph g(si);
    ompl::base::StateSamplerPtr sampler = si->allocStateSampler();
    
    // Add 10 random states and connect them randomly
    for (int i = 0; i < 100; i++)
    {
        ompl::base::State *state = si->allocState();
        sampler->sampleUniform(state);
        g.addVertex(state);
    }
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            if (i==j)
                continue;
            ompl::base::State *iState = boost::get(boost::vertex_prop, g, boost::vertex(i, g)).state;
            ompl::base::State *jState = boost::get(boost::vertex_prop, g, boost::vertex(j, g)).state;
            if (space->distance(iState, jState) < 4)
            {
                if (std::rand() % 4 == 0)
                    g.addEdge(boost::vertex(i, g), boost::vertex(j, g));
            }
        }
    }
    
    std::cout << "Finding at most 10 diverse short paths from node 0 to node 8\n\n";
    Vertex start = boost::vertex(0, g);
    Vertex goal = boost::vertex(8, g);
    std::vector<std::list<Vertex> > paths = findDiverseShortestPaths<StateSpaceNeighborhood>(10, start, goal, g, 0.05);
    std::vector<std::list<Vertex> > paths2 = findDiverseShortestPaths<GraphDistanceNeighborhood>(10, start, goal, g, 0.1);
    std::cout << "Got " << paths.size() << ", " << paths2.size() << " paths\n";
    
    draw(g, start, goal, paths, "output_space.png");
    draw(g, start, goal, paths2, "output_graph.png");
    
    return 0;
    
    // compute dispersion:
    // print min, max, and mean difference between any two paths using Levenshtein edit distance
}

void draw (Graph &g, Vertex start, Vertex goal, std::vector<std::list<Vertex> > paths, const char *filename)
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
    int line_width = 21;
    float red = 1;
    float green = 0.8;
    float blue = 0;
    BOOST_FOREACH(std::list<Vertex> path, paths)
    {
        cairo_set_line_width(cr, line_width);
        cairo_set_source_rgba(cr, red, green, blue, 1);
        Vertex u = path.front();
        BOOST_FOREACH(Vertex v, path)
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
        red -= 1.0/11;
        blue += 1.0/11;
    }
    
    // Draw each vertex as a black dot; except start and goal should be pink
    BOOST_FOREACH(Vertex v, boost::vertices(g))
    {
        ompl::base::State *state = boost::get(boost::vertex_prop, g, v).state;
        double *vcoords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        
        if (v == start || v == goal)
            cairo_set_source_rgba(cr, 1, 0, 1, 1);
            
        cairo_arc(cr, 12+(vcoords[0]+10)*50, 12+(vcoords[1]+10)*50, 8, 0, 2*M_PI);
        cairo_fill(cr);
        cairo_set_source_rgba(cr, 0, 0, 0, 1);
    }
    
    cairo_surface_write_to_png(surface, filename);
}

