""" Implementation of Dijkstra's algorithm for finding the shortest path between nodes in a graph.
    Author: Andres Pulido"""


def dijkstra(graph, start, end):
    """Finds the shortest path between nodes in a graph using Dijkstra's algorithm.

    Args:
        graph: A dictionary representing the graph. The keys are the nodes and the values are dictionaries
            containing the neighbors of the node and the distance to them.
        start: The node to start the search from.

    Returns:
        A dictionary containing the shortest distance to each node from the start node and the path to each node.
    """
    # Initialize the distance and path dictionaries.
    distance = {node: float('inf') for node in graph}
    path = {node: [] for node in graph}
    distance[start] = 0

    # Create a set of unvisited nodes.
    unvisited = set(graph)

    # Loop over the unvisited nodes, stop when visiting the end node
    while unvisited:
        # Get the node with the smallest distance.
        current = min(unvisited, key=distance.get)

        # Remove the current node from the unvisited set.
        unvisited.remove(current)

        # If the current node is the end node, return the distance and path.
        if current == end:
            return distance, path

        # Loop over the neighbors of the current node.
        for neighbor, weight in graph[current].items():
            # Calculate the distance to the neighbor node.
            new_distance = distance[current] + weight

            # If the new distance is less than the current distance, update the distance and path.
            if new_distance < distance[neighbor]:
                distance[neighbor] = new_distance
                path[neighbor] = path[current] + [current]


def main():
    graph = {"A": {"B": 4, "C": 2}, "B": {"A": 4, "C": 1, "D": 5}, "C": {"A": 2, "B": 1, "D": 8, "E": 10},
             "D": {"B": 5, "C": 8, "E": 2, "F": 6}, "E": {"C": 10, "D": 2, "F": 5}, "F": {"D": 6, "E": 5}
    }

    start = "A"
    distance, path = dijkstra(graph, start, end="F")
    #print(f"Shortest distance to each node from {start}: {distance}")
    #rint(f"Shortest path to each node from {start}: {path}")

    print(f"Shortest distance from {start} to F is: {distance['F']}")
    print(f"and the path to get to F is: {path['F'] + ['F']}")


if __name__ == "__main__":
    main()