def print_maze():
    # Print filled maze
    for row in maze:
        print(row)
    print()

def floodfill(maze, start):
    queue = [start]
    visited = set()
    distance = {start: 0}

    while queue:
        current = queue.pop(0)
        visited.add(current)

        print("Current: ", current)
        print()

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
            queue.insert(0,neighbor)
            distance[neighbor] = distance[current] + 1

            maze[neighbor[0]][neighbor[1]] = distance[neighbor]

        print_maze()

    # Fill maze with distance values
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in distance:
                maze[i][j] = distance[(i, j)]
            else:
                maze[i][j] = -1

    return maze


# Example maze
maze = [[0, 0, 0, 0, 0],
        [0, -1, 0, -1, 0],
        [0, -1, 0, -1, 0],
        [0, 0, 0, -1, 0],
        [-1, -1, 0, -1, 0]]

start = (0, 0)

maze = floodfill(maze, start)

