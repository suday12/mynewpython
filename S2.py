# -*- coding: utf-8 -*-
"""
Created on Tue Dec  9 00:10:15 2025

@author: Suday
"""

import heapq


try:
    import networkx as nx
    import matplotlib.pyplot as plt
    HAVE_PLOTTING = True
except Exception:
    HAVE_PLOTTING = False

def dijkstra(graph, start):
    # graph: dict {node: [(neighbor, weight), ...]}
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parent = {node: None for node in graph}

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parent[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))

    return distances, parent


def reconstruct_path(parent, start, target):
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path if path and path[0] == start else None


def visualize_graph(graph, path=None):
    if not HAVE_PLOTTING:
        print("\n(plotting libraries not found — skipping graphical visualization)\n")
        # Simple textual visualization
        print("Graph edges (node -> neighbor : weight):")
        for n in graph:
            for nb, w in graph[n]:
                print(f"{n} -> {nb} : {w}")
        if path:
            print("\nPath edges in order:")
            for a, b in zip(path, path[1:]):
                print(f"{a} -> {b}")
        return

    
    G = nx.Graph()
    for node in graph:
        for neighbor, weight in graph[node]:
            G.add_edge(node, neighbor, weight=weight)

    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True, node_size=700, node_color="skyblue",
            font_size=10, font_weight="bold")
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

    if path:
        edges_in_path = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=edges_in_path,
                               edge_color="red", width=2)

    plt.show()


graph = {
    'A': [('B', 4), ('C', 2)],
    'B': [('A', 4), ('C', 5), ('D', 10)],
    'C': [('A', 2), ('B', 5), ('E', 3)],
    'D': [('B', 10), ('C', 3), ('E', 4)],
    'E': [('D', 4), ('F', 11)],
    'F': [('E', 11)]
}

start, target = 'A', 'F'
distances, parent = dijkstra(graph, start)
path = reconstruct_path(parent, start, target)

print("\n⭐ Shortest distances from source:")
for node, dist in distances.items():
    print(f"{start} -> {node} : {dist if dist != float('inf') else '∞'}")

print(f"\n🔥 Shortest path from {start} to {target}: {path} (Cost = {distances[target]})")

visualize_graph(graph, path)
