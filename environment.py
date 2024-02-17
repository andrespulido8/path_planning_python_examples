"""Defines the workspace to do a path planning comparison."""
import numpy as np
from plot_environment import plot_environment, plot_path

from dijkstra import dijkstra

def exact_cell_decomposition(p0, pf, obstacles, boundary):
    graph = {}

    # Add vertices to the graph
    for obstacle in obstacles:
        for vertex in obstacle:
            graph[tuple(vertex)] = {}
    graph[p0] = {}
    graph[pf] = {}
    graph[boundary[0]] = {}
    graph[boundary[1]] = {}

    # Add edges to the graph
    visited = set() 
    for point in graph:
        for other_point in graph:
            if point != other_point and other_point not in visited:
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
    """
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            p3 = obstacle[i]
            p4 = obstacle[(i + 1) % len(obstacle)]
            if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
                continue
            else:
                if do_lines_intersect(p1, p2, p3, p4):
                    return True
    return False


def do_lines_intersect(p1, p2, p3, p4):
    """ 
    Checks if two line segments intersect.
    Returns True if they intersect, False otherwise.
    """
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    
    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    return intersect(p1, p2, p3, p4)

def main(): 
    p0 = (1, 1)
    pf = (9, 4)

    # boundary of workspace ((x_min, x_max), (y_min, y_max))
    boundary = [(0, 10), (0, 6)]

    # define obstacles by the vertices of the polygon
    B1 = [(3, 3), (3, 4), (5, 4), (5, 3)]
    B2 = [(7, 2), (7, 4), (8, 2)]
    obstacles = [B1, B2]

    graph = exact_cell_decomposition(p0, pf, obstacles, boundary)

    distance, path = dijkstra(graph, p0, pf)

    print(f"Shortest distance from {p0} to {pf} is: {distance[pf]}")
    print(f"and the path to get to pf is: {path[pf] + [pf]}")

    fig, ax = plot_environment(p0, pf, obstacles, boundary)
    plot_path(ax, [node for node in path[pf] + [pf]], p0, pf)

if __name__ == "__main__":
    main()