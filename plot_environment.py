"""Plots the environment for path planning"""
import matplotlib.pyplot as plt
import numpy as np

def plot_environment(ax, start, end, obstacles, limits):
    """ Plots the start and end point, as well as the path, 
        plotted by joining its vertices"""
    ax.set_xlim(limits[0][0], limits[1][0])
    ax.set_ylim(limits[0][1], limits[1][1])
    ax.plot(start[0], start[1], 'ro', markersize=10)
    ax.plot(end[0], end[1], 'b*', markersize=10)
    obstacles = [np.array(obs) for obs in obstacles]
    for obstacle in obstacles:
        # extend the obstacle to connect the first and last vertex
        obstacle = np.vstack([obstacle, obstacle[0]])
        ax.plot(obstacle[:, 0], obstacle[:, 1], 'k')
    return ax

def plot_path(ax, path, start, end):
    """Plots the path on the environment in a green line"""
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], 'g')
    return ax


def plot_graph(ax, graph):
    """Plots the graph"""
    for vertex in graph:
        for other_vertex in graph[vertex]:
            ax.plot([vertex[0], other_vertex[0]], [vertex[1], other_vertex[1]], 'k', alpha=0.05)
    return ax