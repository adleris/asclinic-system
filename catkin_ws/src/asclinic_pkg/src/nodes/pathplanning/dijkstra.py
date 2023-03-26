import numpy as np


# unique_id_assigner stores a unique umber for every vertex
unique_id_assigner = _UniqueIDAssigner()

class Vertex():
    """Stores information about a particular vertex for pathplanning
    
    This class is essentially used to hold the mapping between the (x,y) coordinate
    and the matrix index inside Graph"""
    def __init__(self, x, y):
        global unique_id_assigner
        self.x  = x
        self.y  = y
        self.id = unique_id_assigner.new_id()

    def __lt__(self, other):
        return self.id < other.id
    
    def __eq__(self, other):
        return self.id == other.id

    def __gt__(self, other):
        return self.id > other.id

    def __str__(self):
        return f"Vertex at ({self.x}, {self.y})"


class Edge():
    """Stores information about an edge from one node to another. Edges are
    directed. start and end are the ids of the vertices"""
    def __init__(self, start: int, end: int, weight: float):
        self.start  = start
        self.end    = end
        self.weight = weight

    def __lt__(self, other):
        return self.weight < other.weight

    def __eq__(self, other):
        return self.weight == other.weight

    def __gt__(self, other):
        return self.weight > other.weight


class Graph():
    """A graph to run Dijkstra's algorithm on.
    Graph is assumed to be a DAG with non-negative edge weights
    
    Uses an adjacency matrix internally
    """

    MAX_DIST = 1e9

    def __init__(self, verts: list[Vertex], edges: list[Edge]):
        self.verts   = sorted(verts)
        self.n_verts = len(self.verts)
        self.edges   = sorted(edges)
        self.n_edges = len(self.edges)
        self.matrix  = np.zeros((self.n_verts, self.n_verts))

        self.__populate_graph()

    def __populate_graph(self) -> None:
        """populate graph weights based on the information in self.edges"""
        for edge in self.edges:
            self.matrix[self.edges.start][self.edges.end] = self.edges.weight


    def dijkstra(self, start: int, end: int) -> list[Vertex]:
        """Compute Dijkstra's algorithm on Graph from start to end
        
        start and end are the unique ids associated with each Vertex. The algorithm
        uses the ids to refer to the different Vertices in the graph as indices of
        the adjacency matrix, it is up to you to ensure these ids start from 0."""

        seen = [False for i in range(self.n_verts)]
        distances = np.ones(self.n_verts) * self.MAX_DIST
        distances[start] = 0

        for vert in range(self.verts):
            min_dist = MAX_DIST

            # find the minimum distance edge that we haven't seen yet

        return None


class _UniqueIDAssigner():
    """Class to manage unique id numbers for different instances"""
    def __init__(self):
        # -1 so first id is 0
        self.next = -1
    
    def new_id(self) -> int:
        """Create a new id"""
        self.next += 1
        return self.next