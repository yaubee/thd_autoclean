from shapely.geometry import Point, Polygon
import numpy as np

extent1 = np.array([-0.17428916128973915, -0.07588827131992087, 0.290800005197525])[:2]
extent2 = np.array([-0.09727335936182968, -0.056250697427524345, 0.145])[:2]

# Determine the coordinates of the four corners of the rectangle
min_coords = np.minimum(extent1, extent2)
max_coords = np.maximum(extent1, extent2)

corners = [
    min_coords,
    [min_coords[0], max_coords[1]],
    [max_coords[0], min_coords[1]],
    max_coords
]

rectangle = Polygon(corners)

point1 = np.array([-0.169,-0.06])
point2 = np.array([-0.16, -0.06])

distance = np.linalg.norm(point1 - point2)
largest_distance = max([np.linalg.norm(point1 - np.array(corner)) for corner in corners])
#is_within = np.all((point1 >= min_coords) & (point1 <= max_coords))
is_within = rectangle.contains(Point(point1)) and rectangle.contains(Point(point2))

accuracy = int(is_within)*((largest_distance-distance)/largest_distance)
print(accuracy)


"""
Nearest point query:
"""
# List of coordinates
coordinates = [
    np.array([-0.17308295, -0.12089711,  0.21514407]),
    np.array([-0.18176481, -0.12023317,  0.21374701]),
    np.array([-0.1736979 , -0.11810492,  0.21088994]),
    np.array([-0.16502063, -0.11904474,  0.21210658])
]
query_point = np.array([-0.17, -0.12, 0.21])
distances = [np.linalg.norm(query_point - point) for point in coordinates]


min_index = np.argmin(distances)
nearest_point = coordinates[min_index]

print("Query point:", query_point)
print("Nearest point:", nearest_point)
print("Distance to nearest point:", distances[min_index])