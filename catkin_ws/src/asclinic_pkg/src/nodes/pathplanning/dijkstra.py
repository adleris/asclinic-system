import numpy as np
import heapq

class UniqueIDAssigner():
    """Class to manage unique id numbers for different instances"""
    def __init__(self):
        # -1 so first id is 0
        self.next = -1
    
    def new_id(self) -> int:
        """Create a new id"""
        self.next += 1
        return self.next

# unique_id_assigner stores a unique umber for every vertex
unique_id_assigner = UniqueIDAssigner()

class Vertex():
    """Stores information about a particular vertex for pathplanning
    
    This class is essentially used to hold the mapping between the (x,y) coordinate
    and the matrix index inside Graph"""
    def __init__(self, x, y):
        # the following parameters are constant across all Dijkstra runs
        global unique_id_assigner
        self.x  = x
        self.y  = y
        self.id = unique_id_assigner.new_id()

        # cost updates on a per-run basis
        self._cost = 0

    def __lt__(self, other):
        return self._cost < other._cost
    
    def __eq__(self, other):
        return self._cost == other._cost

    def __gt__(self, other):
        return self._cost > other._cost

    def __str__(self):
        return f"Vertex at ({self.x}, {self.y})"
    
    def __repr__(self):
        return f"Vertex(x:{self.x}, y:{self.y}, id:{self.id}, _cost:{self._cost})"


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

    def __repr__(self):
        return f"{self.start}-->{self.end} ({self.weight})"


class Graph():
    """A graph to run Dijkstra's algorithm on.
    Graph is assumed to be a DAG with non-negative edge weights
    
    Uses an adjacency matrix internally
    """

    MAX_DIST = 1e9
    NO_PARENT = -1

    def __init__(self, verts: list[Vertex], edges: list[Edge]):
        # sort the vert array on id number
        self.verts   = sorted(verts, key = lambda a: a.id)
        self.n_verts = len(self.verts)
        self.edges   = sorted(edges)
        self.n_edges = len(self.edges)

    def dijkstra(self, start: int, end: int) -> list[tuple[float, float]]:
        """Compute Dijkstra's algorithm on Graph from start to end
        
        start and end are the unique ids associated with each Vertex. The algorithm
        uses the ids to refer to the different Vertices in the graph as indices of
        the adjacency matrix, it is up to you to ensure these ids start from 0."""

        # initialise list of distances to each nodes, and the previous node to each
        distances = np.ones(self.n_verts) * self.MAX_DIST
        prev = [self.NO_PARENT for i in range(self.n_verts)]

        # distance to start is defined to be zero
        distances[start] = 0


        # create a priority queue of all of the nodes in the map
        unseen_verts = [v for v in self.verts]
        for i in range(len(unseen_verts)):
            unseen_verts[i]._cost = self.MAX_DIST
        unseen_verts[start]._cost = 0 # TODO: Maybe this is uneccesary
        heapq.heapify(unseen_verts)


        # iterate over every vertex in the graph
        while len(unseen_verts) > 0:
            next_vert = heapq.heappop(unseen_verts)

            # loop through every edge, and select the edges that radiate from this vertex
            for edge in self.edges:
                if edge.start == next_vert.id:

                    # see if this edge is unseen, and gives a shorter path to end
                    if self._vertex_id_is_in_list(edge.end, unseen_verts) and distances[edge.start] + edge.weight < distances[edge.end]:
                        distances[edge.end] = distances[edge.start] + edge.weight
                        prev[edge.end] = edge.start

                        # update unseen_verts with the new distance
                        self._update_heap(unseen_verts, edge.end, distances[edge.end])

            # print(unseen_verts)

        print(distances)
        print(prev)

        print("--------\nSolution Found\n")
        print("Path:")
        prev_vert: int = end
        path = []
        while prev_vert != self.NO_PARENT:
            path.append(prev_vert)
            prev_vert = prev[prev_vert]

        path_str = "->".join((str(p) for p in path[::-1])) + f" cost: {distances[end]}"

        print(path_str)

        # return self._dijkstra_to_coords(prev)
    
    @staticmethod
    def _update_heap(heap: list[Vertex], vert: int, cost: float) -> None:
        """update the cost of a vertex in the heap with the new cost"""
        for i in range(len(heap)):
            if heap[i].id == vert:
                heap[i]._cost = cost
                break

        return heapq.heapify(heap)

    @staticmethod
    def _vertex_id_is_in_list(id: int, vertex_list: list[Vertex]) -> bool:
        """check if there is a vertex with the given id vertex_list"""
        for v in vertex_list:
            if v.id == id:
                return True
        return False
        

    # def _dijkstra_to_coords(self, prev: list[int]) -> list[tuple[float, float]]:
    #     """Convert a list of vertex ids to vertex coordinates

    #     TODO: This should be able to be an linear search since the verts can be sorted, but come back to this later

    #     TODO: I think this is all wrong since we would need start and end here"""

    #     coord_list = []
    #     # scan through the prev list and convert to coords
    #     for p in prev:
    #         for vert in self.verts:
    #             if vert.id == p:
    #                 coord_list.append((vert.x, vert.y))
    #     return coord_list

