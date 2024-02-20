"""Plots the environment for path planning"""
import matplotlib.pyplot as plt
import numpy as np

def plot_environment(ax, start, end, obstacles, limits, algorithm):
    """ Plots the start and end point, as well as the path, 
        plotted by joining its vertices"""
    ax.set_xlim(limits[0][0], limits[1][0])
    ax.set_ylim(limits[0][1], limits[1][1])
    ax.plot(start[0], start[1], 'ro', markersize=10, label="Start")
    ax.plot(end[0], end[1], 'b*', markersize=10, label="Goal")
    obstacles = [np.array(obs) for obs in obstacles]
    for ii, obstacle in enumerate(obstacles):
        # extend the obstacle to connect the first and last vertex
        obstacle = np.vstack([obstacle, obstacle[0]])
        ax.plot(obstacle[:, 0], obstacle[:, 1], 'k')
        # label the obstacle with LateX
        centroid = np.mean(obstacle, axis=0)
        ax.text(centroid[0] - 0.2, centroid[1], f"$CO_{ii}$", fontsize=12)
    ax.title.set_text(f"{algorithm} Path Planning")
    ax.legend()
    return ax

def plot_path(ax, path):
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


def plot_field(ax, f, limits):
    """Plots the potential field
    f: function that returns the force vector at a given point"""
    x = np.linspace(limits[0][0], limits[2][0], 20)
    y = np.linspace(limits[0][1], limits[1][1], 20)
    print("x: and y: ", x, y)
    X, Y = np.meshgrid(x, y)
    U = np.zeros_like(X)
    V = np.zeros_like(Y)
    for i in range(len(x)):
        for j in range(len(y)):
            force = - np.sum(f((x[i], y[j])), axis=0) 
            force = force / np.linalg.norm(force)
            U[j, i] = force[0] 
            V[j, i] = force[1]
    ax.quiver(X, Y, U, V, color="C0", alpha=0.5, scale=10, scale_units="inches", label="Force field") 
    print("Force field plotted. Close the plot to continue. ")
    return ax
