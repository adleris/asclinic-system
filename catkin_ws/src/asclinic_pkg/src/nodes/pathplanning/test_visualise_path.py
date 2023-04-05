from visualise_path import ImageManager
from node_planning import load_coords

im = ImageManager('room-map.png')

# im.new_image([1.5,1.5], [5,3], [(5, 3), (3.5, 5), (2.5, 5), (1.5, 3.5), (1.5, 1.5)])

map_coords = load_coords()

im.new_image(map_coords[0], map_coords[-1], map_coords)