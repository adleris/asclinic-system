from visualise_path import ImageManager
from node_planning import load_coords, load_edges

im = ImageManager('room-map.png')

print("TESTING PATH VISUALISATION")
# print("TEST ONE")
# im.new_image([1.5,1.5], [5,3], [(5, 3), (3.5, 5), (2.5, 5), (1.5, 3.5), (1.5, 1.5)])

# print("TEST TWO")
# map_coords = load_coords(as_objects=False)
# im.new_image(map_coords[0], map_coords[-1], map_coords)

print("DISPLAYING MAP")
map_verts = load_coords(as_objects=True)
map_edges = load_edges(as_objects=True)

# print("Vertices only:")
# im.show_whole_map(map_verts, [])

print("Vertices and edges:")
im.show_whole_map(map_verts, map_edges)