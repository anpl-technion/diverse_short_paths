#!/usr/bin/python


import networkx
import sys

def main():
    
    if len(sys.argc) != 4:
        print("./draw.py <in.graphml> <paths.txt> <out.png>")
        return
    
    # Read the file
    G = networkx.read_graphml(sys.argv[1])
    for k in ['space', 'start', 'goal']:
        if k in G.graph:
            del G.graph[k]
    
    # Read the path file
    f = open(sys.argv[2], "r")
    paths = map(lambda s: s.split(' '), f.read().splitlines())
    f.close()
    
    # Save node coordinates
    for n in G.nodes_iter():
        node = G.node[n]
        coords = map(float, node['coords'].split(','))
        node['xcoord'], node['ycoord'] = coords[0], coords[1]
        del node['coords']
        i = path_index(paths, n)
        if i >= 0:
            node['r'] = 150
            node['g'] = 250
            node['b'] = 100
        else:
            node['r'] = 180
            node['g'] = 180
            node['b'] = 180
        node['size'] = 1.5
    
    # Draw it

def path_index(paths, u):
    
    ret = -1
    for p in paths:
        ret += 1
        for i in xrange(len(p)):
            if p[i] == u:
                return ret
    return -1

if __name__=="__main__":
    main()
