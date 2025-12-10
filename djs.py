import heapq

def dijkstra(graph, start):
    # graph: dict {node: [(neighbor, weight), ...]}
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parent = {node: None for node in graph}

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        # If we popped an outdated entry, skip it
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
    # Walk backwards from target to start using parent links
    while node is not None:
        path.append(node)
        node = parent.get(node)
    path.reverse()
    # Validate that the path actually starts at the start node
    return path if path and path[0] == start else None


def textual_visualize(graph, path=None):
    print("\nGraph edges (node -> neighbor : weight):")
    for n in sorted(graph):
        for nb, w in graph[n]:
            print(f"  {n} -> {nb} : {w}")

    if path:
        print("\nEdges on the chosen shortest path (in order):")
        for a, b in zip(path, path[1:]):
            print(f"  {a} -> {b}")


if __name__ == "__main__":
    # Example graph (undirected edges represented both ways)
    graph = {
        'A': [('B', 4), ('C', 2)],
        'B': [('A', 4), ('C', 5), ('D', 10)],
        'C': [('A', 2), ('B', 5), ('E', 3)],
        'D': [('B', 10), ('C', 3), ('E', 4)],
        'E': [('C', 3), ('D', 4), ('F', 11)],
        'F': [('E', 11)]
    }

    start, target = 'A', 'F'
    distances, parent = dijkstra(graph, start)
    path = reconstruct_path(parent, start, target)

    print("\n⭐ Shortest distances from source:")
    for node in sorted(distances):
        dist = distances[node]
        print(f"{start} -> {node} : {dist if dist != float('inf') else '∞'}")

    if path:
        print(f"\n🔥 Shortest path from {start} to {target}: {path} (Cost = {distances[target]})")
    else:
        print(f"\n⚠️ No path found from {start} to {target}.")

    textual_visualize(graph, path)
