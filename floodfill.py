import serial
import math
from pprint import pprint

arduino = None

try:
    arduino = serial.Serial('COM10', 9600)
except:
    raise Exception("No se ha podido conectar al puerto")

# while True:
#     if arduino.in_waiting > 0:
#         line = arduino.readline().decode().strip()

def print_maze():
    # Print filled maze
    for row in maze:
        print(row)
    print()

def process_sensor_data(sensor_data):
    return {
        "IRL": int(sensor_data[1]),
        "IRD": int(sensor_data[3]),
        "IRF": int(sensor_data[5]),
    }, float(sensor_data[7])

def ir_values_to_walls(sensor_dict, ir_threshold, phi):
    if(phi == 0):
        return {
            "up": sensor_dict["IRL"] < ir_threshold,
            "left": False,
            "down": sensor_dict["IRD"] < ir_threshold,
            "right": sensor_dict["IRF"] < ir_threshold,
        }
    elif(phi == math.pi / 2):
        return {
            "up": sensor_dict["IRF"] < ir_threshold,
            "left": sensor_dict["IRL"] < ir_threshold,
            "down": False,
            "right": sensor_dict["IRD"] < ir_threshold,
        }
    elif(phi == math.pi):
        return {
            "up": sensor_dict["IRD"] < ir_threshold,
            "left": sensor_dict["IRF"] < ir_threshold,
            "down": sensor_dict["IRL"] < ir_threshold,
            "right": False,
        }
    elif(phi == math.pi * (3/2)):
        return {
            "up": False,
            "left": sensor_dict["IRD"] < ir_threshold,
            "down": sensor_dict["IRF"] < ir_threshold,
            "right": sensor_dict["IRL"] < ir_threshold,
        }

def getArePointsAdyacent(p1,p2):
    # d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
    distance= math.sqrt( (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 )
    return distance==1

def getShortestPath(maze, start, end):
    queue = [start]
    visited = set()
    parent = {}  # Map of child: parent

    while queue:
        current = queue.pop(0)
        if current == end:
            # Build path
            path = [end]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            return path
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
            parent[neighbor] = current

    return None

def floodfill(maze, start, end):
    original_maze = maze.copy()
    
    queue = [start]
    visited = set()
    distance = {start: 0}
    parent = {}  # Map of child: parent

    last_visited = None
    
    while queue:
        current = queue.pop(0)
        
        print("Current: ", current)
        
        print(last_visited, current)
        
        if last_visited != None and last_visited != current:
            arePointsAdyacent = getArePointsAdyacent(current, last_visited)
            if(arePointsAdyacent):
                
                arduinoMovesTo = (
                    last_visited[0]-current[0],
                    current[1]-last_visited[1]
                )
                
                print("Para llegar a esta posición, el arduino se tiene que mover a (",arduinoMovesTo[0],",",arduinoMovesTo[1], ")")
                
                arduino.write((str(arduinoMovesTo[0])+"\n").encode())
                arduino.write((str(arduinoMovesTo[1])+"\n").encode())
                while True:
                    if arduino.in_waiting > 0:
                        line = arduino.readline().decode().strip()
                        data = line.split(",")
                        sensor_dict, phi = process_sensor_data(data)
                        walls = ir_values_to_walls(sensor_dict, 50, phi)
                        
                        # if walls['left']:
                        #     maze[current[0]][current[1]-1] = -1
                        # if walls['right']:
                        #     maze[current[0]][current[1]+1] = -1
                        # if walls['up']:
                        #     maze[current[0]-1][current[1]] = -1
                        # if walls['down']:
                        #     maze[current[0]+1][current[1]] = -1
                        
                        print(line)
                        break
            else:
                path = getShortestPath(original_maze,last_visited,current)
                followPath(path)
                
        last_visited = current
           
        # Genera error
        # if(current in visited):
        #     continue 
        
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
            
            maze[neighbor[0]][neighbor[1]] = distance[neighbor]
            
            if neighbor == end:
                # Build path
                path = [end]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                path.reverse()
                print_maze()
                return maze, path
            
        print_maze()
        
        # input("Presione enter para continuar")

    # Fill maze with distance values
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if (i, j) in distance:
                maze[i][j] = distance[(i, j)]
            else:
                maze[i][j] = -1

    return maze, None

def followPath(path):
    last_visited = None
    
    while path:
        current = path.pop(0)
        
        print("Current: ", current)
        
        print(last_visited, current)
        
        if(last_visited != current and last_visited != None):
            
            arduinoMovesTo = (
                last_visited[0]-current[0],
                current[1]-last_visited[1]
            )
            
            print("Para llegar a esta posición, el arduino se tiene que mover a (",arduinoMovesTo[0],",",arduinoMovesTo[1], ")")
            
            arduino.write((str(arduinoMovesTo[0])+"\n").encode())
            arduino.write((str(arduinoMovesTo[1])+"\n").encode())
            while True:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode().strip()
                    print(line)
                    break
                
        last_visited = current

# Example maze
maze = [[0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, -1, 0, 0, 0],
        [0, 0, 0, -1, 0],
        [-1, -1, 0, -1, 0]]

# 25 * 15
maze = [
    [-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1],
]

maze = [
    [-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	-1,	-1,	-1,	-1,	-1,	0,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	0,	-1,	-1,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	-1,	0,	0,	0,	0,	0,	0,	0,	-1,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	-1,	-1,	-1,	0,	-1,	-1,	-1,	0,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	0,	-1,	0,	-1,	-1,	-1,	0,	-1],	
    [-1,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	-1],	
    [-1,	0,	-1,	0,	-1,	-1,	-1,	0,	-1,	0,	-1,	0,	-1,	0,	-1,	0,	-1,	-1,	-1,	-1,	-1,	0,	-1,	0,	-1],	
    [-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	-1,	0,	0,	0,	0,	0,	-1,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	-1,	-1,	-1,	0,	-1,	-1,	-1,	0,	-1,	-1,	-1,	-1,	-1,	0,	-1,	-1,	-1,	0,	-1,	-1,	-1,	0,	-1],	
    [-1,	0,	0,	0,	0,	0,	0,	0,	-1,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	0,	0,	-1,	0,	-1],	
    [-1,	0,	-1,	-1,	-1,	-1,	-1,	0,	-1,	0,	-1,	0,	-1,	-1,	-1,	-1,	-1,	0,	-1,	-1,	-1,	0,	-1,	0,	-1],	
    [-1,	0,	-1,	0,	0,	0,	0,	0,	-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	0,	-1,	0,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	0,	-1,	-1,	-1],	
    [-1,	0,	-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	-1],	
    [-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1],
]

# Verify maze size
# print(len(maze))
# for row in maze:
#     print(len(row))

# input()

start = (1, 1)
end = (7, 13)

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
    
print("Segunda fase, posicione al robot en la posición inicial y presione enter.")

input()

followPath(path)