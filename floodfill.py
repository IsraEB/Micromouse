def floodfill(maze, start, end):
    queue = [start]
    visited = set()
    distance = {start: 0}
    parent = {}  # Map of child: parent

    while queue:
        current = queue.pop(0)
        visited.add(current)

        # Check neighbors
        neighbors = [(current[0]-1, current[1]),  # North
                     (current[0]+1, current[1]),  # South
                     (current[0], current[1]-1),  # West
                     (current[0], current[1]+1)]  # East
        for neighbor in neighbors:
            if neighbor[0] < 0 or neighbor[0] >= len(maze) or \
               neighbor[1] < 0 or neighbor[1] >= len(maze[0]) or \
               neighbor in visited or maze[neighbor[0]][neighbor[1]] == -1:
                continue
            queue.append(neighbor)
            visited.add(neighbor)
            distance[neighbor] = distance[current] + 1
            parent[neighbor] = current
            if neighbor == end:
                # Build path
                path = [end]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                return maze, path

    # Fill maze with distance values
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in distance:
                maze[i][j] = distance[(i, j)]
            else:
                maze[i][j] = -1

    return maze, None


# Example maze
maze = [[0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, 0, 0, -1, 0],
        [-1, -1, 0, -1, 0]]

start = (0, 0)
end = (0, 2)

maze, path = floodfill(maze, start, end)

if path:
    print("Shortest path found:", path)
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == -1:
                print("#", end="")
            elif (i, j) in path:
                print(".", end="")
            else:
                print(maze[i][j], end="")
        print()
else:
    print("No path found.")