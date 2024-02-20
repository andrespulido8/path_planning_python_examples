""" this module contains the visibility graph algorithm 
"""
import numpy as np

def visibility_graph(p0, pf, obstacles, limits):
    graph = {}

    # Add vertices to the graph
    obs_id = {}
    for ii, obstacle in enumerate(obstacles):
        for vertex in obstacle:
            obs_id[vertex] = ii
            graph[tuple(vertex)] = {}
    for limit in limits:
        graph[limit] = {}
    graph[p0] = {}
    graph[pf] = {}

    # Add edges to the graph
    visited = set() 
    for point in graph:
        for other_point in graph:
            if point in obs_id and other_point in obs_id and obs_id[point] == obs_id[other_point]:
                    continue
            elif point != other_point and other_point not in visited:
                if not is_line_intersecting(point, other_point, obstacles):
                    distance = np.linalg.norm(np.array(point) - np.array(other_point))
                    graph[point][other_point] = graph[other_point][point] = distance 
        visited.add(point)
    
    return graph

def is_line_intersecting(p1, p2, obstacles):
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

    Source: Github Copilot
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
                    return True
    return False
