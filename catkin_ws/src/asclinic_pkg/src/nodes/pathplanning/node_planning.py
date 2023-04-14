"""file to parse out data points into verticies for the map

This file contains the exports for:

- path planning vertex locations
- edges between vertices
"""

from dijkstra import Edge, Vertex

def load_coords():
    """Load in coordinates of path planning vertices

    The vertices are returned in x, y coordinates, following the coordinate scheme as outlined elsewhere"""

    # for 'room-map.png'
    ROOM_MAP_CM_TO_PIXELS = 0.56

    # for 'room_map_node_planning.jpeg'
    NODE_PLANNING_TO_ROOM_MAP_CM_TO_PIXELS = 0.7628 # cm / pixel

    # positions of the nodes in PIXELS as given by the file 'room_map_node_planning.jpeg'
    nodes =[
        [135, 100],
        [316, 68],
        [500, 96],
        [720, 71],
        [878, 130],
        [1100, 106],
        [1220, 140],
        [100, 240],
        [510, 285],
        [858, 266],
        [1220, 305],
        [60, 400],
        [207, 467],
        [630, 385],
        [820, 389],
        [1253, 445],
        [330, 600],
        [540, 527],
        [860, 545],
        [1024, 590],
        [1200, 640],
        [240, 740],
        [510, 723],
        [600, 864],
        [800, 920],
        [950, 793],
        [1110, 720],
        [60, 960],
        [175, 1000],
        [596, 1016],
        [812, 1065],
        [1245, 890],
        [1220, 1007],
        [137, 1185],
        [270, 1240],
        [465, 1211],
        [666, 1240],
        [687, 1142],
        [832, 1197],
        [1056, 1214],
        [1200, 1180]
    ]

    # convert pixel values into xy coordinates (expressed in metres)
    vertex_coords: list[tuple[int, int]] = [(0,0) for n in range(len(nodes))]
    for n in range(len(nodes)):
        vertex_coords[n] = (nodes[n][0] * NODE_PLANNING_TO_ROOM_MAP_CM_TO_PIXELS /100, 
                            nodes[n][1] * NODE_PLANNING_TO_ROOM_MAP_CM_TO_PIXELS /100)

    # return vertex_coords

    return __coord_list_to_Vertex(vertex_coords)

def load_edges():
    edges: list[list[int, int, float]] = [
        # from 0
        [0, 1, 1],
        [0, 7, 1],
        # from 1
        [1, 2, 1],
        # from 2
        [2, 3, 1],
        [2, 8, 1],
        # from 3
        [3, 4, 1],
        # from 4
        [4, 5, 1],
        [4, 9, 1],
        # from 5
        [5, 6, 1],
        # from 6
        [6, 10, 1],
        # from 7
        [7, 11, 1],
        # from 8
        [8, 13, 1],
        [8, 17, 1],
        # from 9
        [9, 14, 1],
        # from 10
        [10, 15, 1],
        # from 11
        [11, 12, 1],
        # from 12
        [12, 16, 1],
        [12, 21, 1],
        # from 13
        [13, 14, 1],
        [13, 17, 1],
        # from 14
        [13, 19, 1],
        # from 15
        [15, 20, 1],
        # from 16
        [16, 17, 1],
        [16, 18, 1],
        [16, 21, 1],
        [16, 22, 1],
        # from 17
        [17, 18, 0.3],
        [18, 21, 1],
        # from 18

        # from 19
        [19, 20, 1],
        [19, 25, 1],
        [19, 26, 1],
        # from 20
        [20, 26, 1],
        [20, 31, 1],
        # from 21
        [21, 27, 1],
        [21, 22, 1],
        # from 22
        [22, 23, 1],
        # from 23
        [23, 24, 1],
        [23, 29, 1],
        # from 24
        [24, 25, 1],
        [24, 30, 1],
        # from 25
        [25, 26, 1],
        # from 26
        [26, 31, 1],
        # from 27
        [27, 28, 1],
        [27, 33, 1],
        # from 28
        [28, 33, 1],
        # from 29
        [29, 30, 1],
        [29, 25, 1],
        [29, 37, 1],
        # from 30
        [30, 37, 1],
        # from 31
        [31, 32, 1],
        # from 32
        [32, 40, 1],
        # from 33
        [33, 34, 1],
        # from 34
        [34, 35, 1],
        # from 35
        [35, 36, 1],
        [35, 37, 1],
        # from 36
        [36, 38, 1],
        # from 37
        [37, 38, 1],
        # from 38
        [38, 39, 1],
        # from 39
        [39, 40, 1]
        # from 40
    ]
    # return edges
    return __edge_list_to_Edges(edges)


def __coord_list_to_Vertex(verts: list[tuple[int, int]]):
    return [Vertex(v[0], v[1]) for v in verts]

def __edge_list_to_Edges(edges: list[list[int, int, float]]):
    return [Edge(e[0], e[1], e[2]) for e in edges]