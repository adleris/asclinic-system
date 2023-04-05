"""file to parse out data points into verticies for the map

This file contains the exports for:

- path planning vertex locations
- edges between vertices
"""

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
        [560, 545],
        [1024, 590],
        [1200, 640],
        [240, 740],
        [510, 723],
        [600, 834],
        [860, 885],
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

    return vertex_coords