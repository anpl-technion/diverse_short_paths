/*
 * DStar.cpp
 */

#include "DStar.h"

DStar::DStar (const Graph &g, const Vertex start, const Vertex goal)
  : g(g), start(start), goal(goal), last(start),
    costs(g.getNumVertices(), {std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::infinity()}),
    keyModifier(0), avoiding(), updatedEdges()
{
    costs[goal].rhs = 0;
    pq.insert(BimapValue(goal, calculateKey(goal)));
    computeShortestPath();
}

DStar::DStar (const DStar *old, const Neighborhood &avoid)
  : g(old->getGraph()), start(old->getStart()), goal(old->getGoal()),
    last(start), pq(old->getPQ()), costs(old->getCosts()),
    keyModifier(old->getKeyModifier()), avoiding(old->getAvoiding()), updatedEdges()
{
    BOOST_FOREACH(Edge e, boost::edges(g))
    {
        if (avoiding.count(e) == 0 && avoid.shouldAvoid(e))
        {
            Vertex u = boost::source(e, g);
            Vertex v = boost::target(e, g);
            Edge e2 = boost::edge(v, u, g).first;
            avoiding.insert(e);
            avoiding.insert(e2);
            updatedEdges.insert(e);
            updatedEdges.insert(e2);
        }
    }
    replan(start);
}

const Graph &DStar::getGraph () const
{
    return g;
}

Vertex DStar::getStart () const
{
    return start;
}

Vertex DStar::getGoal () const
{
    return goal;
}

DStar::Bimap DStar::getPQ () const
{
    return pq;
}

std::vector<DStar::cost> DStar::getCosts () const
{
    return costs;
}

double DStar::getKeyModifier () const
{
    return keyModifier;
}

std::set<Edge> DStar::getAvoiding() const
{
    return avoiding;
}

double DStar::edgeWeight (const Edge e) const
{
    if (avoiding.count(e) == 0)
        return g.getEdgeWeight(e);
    return std::numeric_limits<double>::infinity();
}

DStar::Key DStar::calculateKey (const Vertex v) const
{
    const cost &c = costs[v];
    double min = std::min(c.rhs, c.g);
    return std::make_pair(min + h(start, v) + keyModifier, min);
}

void DStar::updateVertex(const Vertex v)
{
    cost &c = costs[v];
    if (v != goal)
    {
        double min = std::numeric_limits<double>::infinity();
        BOOST_FOREACH(Edge e, boost::out_edges(v, g)) 
        {
            min = std::min(min, edgeWeight(e) + costs[target(e, g)].g);
        }
        c.rhs = min;
    }

    auto it = pq.left.find(v);
    if (it != pq.left.end()) pq.left.erase(it);

    if (c.g != c.rhs) 
        pq.insert(BimapValue(v, calculateKey(v)));
}

void DStar::computeShortestPath() 
{
    while (!pq.empty() && (pq.right.begin()->first < calculateKey(start) || 
            costs[start].rhs != costs[start].g))
    {
        auto it = pq.right.begin();
        Key kold = it->first;
        Vertex v = it->second;
        pq.right.erase(it);

        Key k = calculateKey(v);
        cost &cv = costs[v];

        if (kold < k)
        {
            pq.insert(BimapValue(v, k));
        }
        else if (cv.g > cv.rhs)
        {
            cv.g = cv.rhs;
            BOOST_FOREACH(Vertex v, boost::adjacent_vertices(v, g))
            {
                updateVertex(v);
            }
        }
        else
        {
            cv.g = std::numeric_limits<double>::infinity();
            updateVertex(v);
            BOOST_FOREACH(Vertex v, boost::adjacent_vertices(v, g))
            {
                updateVertex(v);
            }
        }
    }
}

double DStar::h(const Vertex u, const Vertex v) const
{
    return g.getSpaceInfo()->distance(g.getVertexState(u), g.getVertexState(v));
}
        

Path DStar::getPath() const
{
    Path path(&g);
    if (pq.empty() || costs[last].g == std::numeric_limits<double>::infinity())
        return path;

    path.push_back(last);

    Vertex v = last;
    while (v != goal)
    {
        double min = std::numeric_limits<double>::infinity();
        Vertex best_v = v;
        BOOST_FOREACH(Edge e, boost::out_edges(v, g))
        {
            Vertex v = boost::target(e, g);
            double c = edgeWeight(e) + costs[v].g;
            if (c <= min)
            {
                min = c;
                best_v = v;
            }
        }
        v = best_v;
        if (v == 0)
            return Path(&g);
        path.push_back(v);
    }

    return path;
}

void DStar::replan(const Vertex v) 
{
    keyModifier = keyModifier + h(v, last);
    last = v;

    BOOST_FOREACH(Edge e, updatedEdges)
    {
        Vertex u, v;
        g.getVertices(e, &u, &v);
        updateVertex(u);
        updateVertex(v);
    }
    computeShortestPath();
}
