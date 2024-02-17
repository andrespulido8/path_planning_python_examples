"""Plots the environment for path planning"""
import matplotlib.pyplot as plt
import numpy as np

def plot_environment(start, end, obstacles, boundary):
    """ Plots the environment with a star at the start, a circle
    at the end, and the polygons it joins them with the vertices"""
    fig, ax = plt.subplots()
    ax.set_xlim(boundary[0][0], boundary[0][1])
    ax.set_ylim(boundary[1][0], boundary[1][1])
    ax.plot(start[0], start[1], 'r*', markersize=10)
    ax.plot(end[0], end[1], 'bo', markersize=10)
    obstacles = [np.array(obs) for obs in obstacles]
    for obstacle in obstacles:
        # extend the obstacle to connect the first and last vertex
        obstacle = np.vstack([obstacle, obstacle[0]])
        ax.plot(obstacle[:, 0], obstacle[:, 1], 'k')
    return fig, ax


def plot_path(ax, path, start, end):
    """Plots the path on the environment in a green line"""
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], 'g')
    plt.show()
    print("Path plotted.")
    return ax