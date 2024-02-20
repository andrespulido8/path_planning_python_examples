import numpy as np

from exact_cell_decomposition import is_line_intersecting

def PRM(p0, pf, obstacles, limits):
    """ Probabilistic Roadmap Method
        p0: start point
        pf: end point
        obstacles: list of obstacles, each obstacle is a list of vertices
        limits: boundary of the workspace
    """
    graph = {}
    # add the start and end points to the graph
    graph[p0] = {}
    graph[pf] = {}
    # add the limits to the graph
    for limit in limits:
        graph[limit] = {}
    sampled_points = 50
    for i in range(sampled_points):
        # sample a point in the workspace
        sample = np.random.uniform(limits[0], limits[2])
        if not is_inside_obstacle(sample, obstacles):
            graph[tuple(sample)] = {}
            # connect it to the k-nearest neighbors
            k = 5
            distances = {}
            for vertex in graph:
                distances[vertex] = np.linalg.norm(sample - vertex)
            distances = {k: v for k, v in sorted(distances.items(), key=lambda item: item[1])}
            for vertex in list(distances.keys())[1:k]:
                if not is_line_intersecting(vertex, tuple(sample), obstacles):
                    graph[tuple(sample)][vertex] = distances[vertex]
                    graph[vertex][tuple(sample)] = distances[vertex]
    return graph


def is_inside_obstacle(point, obstacles):
    """ Checks if a point is inside the obstacles. 
        The obstacles are defined by the vertices of a convex polygon."""
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            p1 = obstacle[i]
            p2 = obstacle[(i + 1) % len(obstacle)]
            p3 = point
            if ccw(p1, p2, p3) != ccw(p1, p2, obstacle[0]):
                return False 
    return True 
