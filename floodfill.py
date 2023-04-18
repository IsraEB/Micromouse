import serial

try:
    arduino = serial.Serial('COM10', 9600)
except:
    print("No se ha podido conectar al puerto")

# while True:
#     if arduino.in_waiting > 0:
#         line = arduino.readline().decode().strip()

def print_maze():
    # Print filled maze
    for row in maze:
        print(row)
    print()

def floodfill(maze, start, end):
    queue = [start]
    visited = set()
    parent = {}  # Map of child: parent Shortest
    distance = {start: 0} # Maze

    shortestPath = None

    while queue:
        current = queue.pop(0)
        if current == end:
            # Build path
            path = [end]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            shortestPath = path

            # return shortestPath, maze

        visited.add(current)

        print("Current: ", current)
        print()

        # Check neighbors
        neighbors = [(current[0]-1, current[1]),  # North
                     (current[0]+1, current[1]),  # South
                     (current[0], current[1]-1),  # West
                     (current[0], current[1]+1)]  # East

        reachable_neighbors_count = 0

        for neighbor in neighbors:
            if neighbor[0] < 0 or neighbor[0] >= len(maze) or \
               neighbor[1] < 0 or neighbor[1] >= len(maze[0]) or \
               neighbor in visited or maze[neighbor[0]][neighbor[1]] == -1:
                continue
            queue.insert(0,neighbor)
            parent[neighbor] = current
            distance[neighbor] = distance[current] + 1

            maze[neighbor[0]][neighbor[1]] = distance[neighbor]

            reachable_neighbors_count = reachable_neighbors_count + 1

        current_is_bifurcation = reachable_neighbors_count > 1

        print_maze()

    # Fill maze with distance values
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in distance:
                maze[i][j] = distance[(i, j)]
            else:
                maze[i][j] = -1

    return shortestPath, maze


# Example maze
maze = [[0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, 0, 0, -1, 0],
        [-1, -1, 0, -1, 0]]

start = (0, 0)
end = (len(maze)-1, len(maze[0])-1)

path, maze = floodfill(maze, start, end)

if path:
    print("Shortest path found:", path)
else:
    print("No path found.")

