""" this module contains the exact_cell_decomposition function
"""
import numpy as np

def exact_cell_decomposition(p0, pf, obstacles, limits):
    # implement polygonal cell decomposition
    vertex_arr = np.array([vertex for obstacle in obstacles for vertex in obstacle])
    # sort the obstacles by the y-coordinate
    vertex_arr = vertex_arr[vertex_arr[:, 1].argsort()]
    # add top left and top right corners of the workspace
    vertex_arr = np.vstack([vertex_arr, limits[1], limits[2]]).tolist()

    polygons = [[list(limits[0]), list(limits[3])]]
    pol_count = 0
    for ii, vertex in enumerate(vertex_arr):
        if vertex in polygons[-1]:
            continue
        else:
            polygons.append([])

            polygons[-1].append(vertex)

            count = 0
            while ii + count  + 1 < len(vertex_arr) and vertex[1] == vertex_arr[ii + count + 1][1]:
                # assuming there are no obstacles blocking this line
                polygons[-1].append(vertex_arr[ii + 1])
                count += 1
        
            # check left
            result, segment = is_line_intersecting_obstacle(tuple(vertex), (limits[0][0], vertex[1]), obstacles)
            while segment is not None and segment[1][1] == vertex[1]:
                result, segment = is_line_intersecting_obstacle(segment[1], (limits[3][0], vertex[1]), obstacles)
            if result:
                # add the intersection point to the polygon
                polygons[-1].append([find_x_intercept(segment[0], segment[1], vertex[1]), vertex[1]])
                # lower left corner of the previous polygon that intersects the obstacle
                polygons[-1].append([find_x_intercept(segment[0], segment[1], polygons[-2][0][1]), polygons[-2][0][1]])  
            else:
                polygons[-1].append([limits[0][0], vertex[1]])
                polygons[-1].append([limits[0][0], polygons[-2][0][1]])  # lower left corner of the previous polygon

            # check right
            result, segment = is_line_intersecting_obstacle(tuple(vertex), (limits[3][0], vertex[1]), obstacles)
            while segment is not None and segment[1][1] == vertex[1]:
                result, segment = is_line_intersecting_obstacle(segment[1], (limits[3][0], vertex[1]), obstacles)
            if result:
                # add the intersection point to the polygon
                polygons[-1].append([find_x_intercept(segment[0], segment[1], vertex[1]), vertex[1]])
                # lower left corner of the previous polygon that intersects the obstacle
                polygons[-1].append([find_x_intercept(segment[0], segment[1], polygons[-2][0][1]), polygons[-2][0][1]])  
                # TODO: keep track of the extra polygon added because of the intersection (use pol_count)
            else:
                polygons[-1].append([limits[3][0], vertex[1]])
                polygons[-1].append([limits[3][0], polygons[-2][0][1]])  # lower right corner of the previous polygon
    
    polygons.pop(0)  # remove the first empty polygon

    graph = {}
    graph[p0] = {}
    graph[pf] = {}

    # include centroids for each polygon to the graph
    for polygon in polygons:
        x = sum([vertex[0] for vertex in polygon]) / len(polygon)
        y = sum([vertex[1] for vertex in polygon]) / len(polygon)
        graph[(x, y)] = {}
    
    # Add edges to the graph
    visited = set() 
    for point in graph:
        for other_point in graph:
            if point != other_point and other_point not in visited:
                if not is_line_intersecting_obstacle(point, other_point, obstacles)[0]:
                    distance = np.linalg.norm(np.array(point) - np.array(other_point))
                    graph[point][other_point] = graph[other_point][point] = distance 
        visited.add(point)

    return graph, polygons

def find_x_intercept(p1, p2, y3):
    x1, y1 = p1
    x2, y2 = p2
    if x1 == x2:
        return x1
    # Calculate slope
    m = (y2 - y1) / (x2 - x1)
    # Calculate y-intercept
    b = y1 - m * x1
    # Calculate x-coordinate of intersection point
    x3 = (y3 - b) / m
    return x3

def is_line_intersecting_obstacle(p1, p2, obstacles):
    """
    Checks if the line between two points intersects any of the obstacles,
    defined by a list of the vertices in a convex polygon.
    Returns True if the line intersects an obstacle, False otherwise.
    
    Parameters:
        p1 (list): List representing the first point [x, y].
        p2 (list): List representing the second point [x, y].
        obstacles (list): List of obstacles, each obstacle is represented
                          by a list of vertices [[x1, y1], [x2, y2], ...].
    
    Returns:
        bool: True if the line intersects an obstacle, False otherwise.
    """
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            p3 = obstacle[i]
            p4 = obstacle[(i + 1) % len(obstacle)]
            if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
                continue
            else:
                if ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4):
                    return (True, [p3, p4])
    return (False, None)
