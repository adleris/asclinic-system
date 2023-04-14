import numpy as np

from dijkstra import *

# construct up a graph and pass in in for testing
verts: list[Vertex] = [
    Vertex(0, 0), # a
    Vertex(0, 1), # b
    Vertex(1, 0), # c
    Vertex(1, 1), # d
    Vertex(2, 0), # e
    Vertex(2, 1)  # f
]

# build up the network of edges
edges: list[Edge] = [
    # from a
    Edge(0, 2, 4),
    Edge(0, 3, 1),
    # from b
    Edge(1, 0, 1),
    # from c
    Edge(2, 4, 2),
    Edge(2, 5, 3),
    # from d
    Edge(3, 1, 2),
    Edge(3, 2, 1),
    Edge(3, 4, 4),
    Edge(3, 5, 5),
    # from e
    
    # from f
    Edge(5, 4, 1)
]

g = Graph(verts, edges)

g.dijkstra(0, 5)