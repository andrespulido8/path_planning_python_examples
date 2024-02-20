"""Defines the workspace to do a path planning comparison."""
import numpy as np
import matplotlib.pyplot as plt
from plot_environment import plot_environment, plot_path, plot_graph, plot_field

from dijkstra import dijkstra
from exact_cell_decomposition import exact_cell_decomposition
from potential_field import potential_field
from prm import PRM


def main(): 
    algorithm = "PRM"

    p0 = (1, 1)
    pf = (9, 4)

    # boundary of workspace ((x_min, y_min), (x_max, y_max))
    boundary = ((0, 0), (10, 6))
    # limits are the four corners of the boundary
    limits = (boundary[0], (boundary[0][0], boundary[1][1]), boundary[1], (boundary[1][0], boundary[0][1])) 

    # define obstacles by the vertices of the polygon
    B1 = ((3, 3), (3, 4), (5, 4), (5, 3))
    B2 = ((7, 2), (7, 4), (8, 2))
    obstacles = (B1, B2)

    if algorithm == "potential_field":
        force_field, path = potential_field(p0, pf, obstacles)
    elif algorithm == "exact_cell_decomposition":
        graph = exact_cell_decomposition(p0, pf, obstacles, limits)
        distance, path = dijkstra(graph, p0, pf)
        print(f"Shortest distance from {p0} to {pf} is: {distance[pf]}")
        print(f"and the path to get to pf is: {path[pf] + [pf]}")
    elif algorithm == "PRM":
        graph = PRM(p0, pf, obstacles, limits)
        distance, path = dijkstra(graph, p0, pf)

    fig, ax = plt.subplots()
    if algorithm == "exact_cell_decomposition" or algorithm == "PRM":
        ax = plot_graph(ax, graph)
        ax = plot_path(ax, [node for node in path[pf] + [pf]])
    elif algorithm == "potential_field":
        ax = plot_path(ax, path)
        ax = plot_field(ax, force_field, limits)
    ax = plot_environment(ax, p0, pf, obstacles, boundary, algorithm)
    plt.show()

if __name__ == "__main__":
    main()