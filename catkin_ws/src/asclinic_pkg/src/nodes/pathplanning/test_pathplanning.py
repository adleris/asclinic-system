import visualise_path
from dijkstra import *

# print("TEST ONE -- simple path")

# (5, 3), (3.5, 5), (2.5, 5), (1.5, 3.5), (1.5, 1.5)

# verts: list[Vertex] = [
#     Vertex(1,1),
#     Vertex(1,2)
# ]

# # build up the network of edges
# edges: list[Edge] = [
#     Edge(0, 1, 1)
#     # # from a
#     # Edge(0, 2, 4),
#     # Edge(0, 3, 1),
#     # # from b
#     # Edge(1, 0, 1),
#     # # from c
#     # Edge(2, 4, 2),
#     # Edge(2, 5, 3),
#     # # from d
#     # Edge(3, 1, 2),
#     # Edge(3, 2, 1),
#     # Edge(3, 4, 4),
#     # Edge(3, 5, 5),
#     # # from e
    
#     # # from f
#     # Edge(5, 4, 1)
# ]

# g = Graph(verts, edges)

# path = g.dijkstra(0, 1)

# # pv = visualise_path.PathVisualiser()

# im = visualise_path.ImageManager('room-map.png')

# im.new_image((2,1), (0, 0), path)


print("TEST TWO -- room map")

import node_planning

verts = node_planning.load_coords()
edges = node_planning.load_edges()

graph = Graph(verts, edges)

path = graph.dijkstra(7, 40)

im = visualise_path.ImageManager('room-map.png')

im.new_image([verts[7].x, verts[7].y], [verts[40].x, verts[40].y], path)