import numpy as np


# unique_id_assigner stores a unique umber for every vertex
unique_id_assigner = _UniqueIDAssigner()

class Vertex():
    """Stores information about a particular vertex for pathplanning"""
    def __init__(self, x, y):
        global unique_id_assigner
        self.x  = x
        self.y  = y
        self.id = unique_id_assigner.new_id()
    
    def __eq__(self, other):
        return self.unique_id_assigner == other.unique_id_assigner

    def __str__(self):
        return f"Vertex at ({self.x}, {self.y})"


class Edge():
    """Stores information about an edge from one node to another. Edges are directed"""
    def __init__(self, start: Vertex, end: Vertex, weight: float):
        self.start  = start
        self.end    = end
        self.weight = weight

    def __lt__(self, other):
        return self.weight < other.weight

    def __eq__(self, other):
        return self.weight == other.weight

    def __gy__(self, other):
        return self.weight > other.weight


class Graph():
    """A graph to run Dijkstra's algorithm on.
    Graph is assumed to be a DAG with non-negative edge weights
    
    Uses an adjacency matrix internally
    """
    def __init__(self, verts: list[Vertex], edges: list[Edge]):
        self.nodes = nodes
        self.matrix = np.zeros((len(nodes), len(nodes)))


    def dijkstra(self, start: Vertex, en: Vertex) -> list[Vertex]:
        """Compute Dijkstra's algorithm on Graph from start to end"""
        return None


class _UniqueIDAssigner():
    """Class to manage unique id numbers for different instances"""
    def __init__(self):
        self.next = 0
    
    def new_id(self) -> int:
        """Create a new id"""
        self.next += 1
        return self.next