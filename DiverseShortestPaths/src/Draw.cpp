
/* Author: Caleb Voss */

#ifndef NO_DRAW

#include "Draw.h"

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cairo/cairo.h>

#include "Graph.h"
#include "Path.h"

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
