import serial
import time
from pprint import pprint

def flood_fill(maze, start, goal_area):
    maze[start[0]][start[1]] = 0
    queue = [start]

    while queue:
        x, y = queue.pop(0)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != -1:
                new_value = maze[x][y] + 1
                if maze[nx][ny] == 0 or maze[nx][ny] > new_value:
                    maze[nx][ny] = new_value
                    queue.append((nx, ny))

    path = []
    x, y = None, None

    min_val = float('inf')
    for gx, gy in goal_area:
        if maze[gx][gy] < min_val:
            min_val, x, y = maze[gx][gy], gx, gy

    while (x, y) != start:
        path.append((x, y))
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        min_val, next_cell = float('inf'), None
        for nx, ny in neighbors:
            if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] != -1 and maze[nx][ny] < min_val:
                min_val, next_cell = maze[nx][ny], (nx, ny)
        x, y = next_cell

    path.reverse()
    return path

def send_phase_change_to_robot(ser):
    command = 'S'
    ser.write(command.encode())
    time.sleep(0.5)

def init_maze(rows, cols):
    maze = [[None] * cols for _ in range(rows)]
    return maze

def ir_values_to_walls(sensor_values, ir_threshold):
    return {
        "left": sensor_values["IRL"] < ir_threshold,
        "right": sensor_values["IRD"] < ir_threshold,
        "front": sensor_values["IRF"] < ir_threshold,
        "back": False
    }

def update_maze(maze, x, y, walls):
    if walls['left']:
        maze[x][y-1] = -1
    if walls['right']:
        maze[x][y+1] = -1
    if walls['front']:
        maze[x-1][y] = -1
    if walls['back']:
        maze[x+1][y] = -1

def process_sensor_data(sensor_data, current_position):
    sensor_values = sensor_data
    sensor_dict = {
        "IRL": int(sensor_values[1]),
        "IRD": int(sensor_values[3]),
        "IRF": int(sensor_values[5]),
        "SR1": int(sensor_values[7]),
        "SR2": int(sensor_values[9])
    }
    return sensor_dict, current_position

def read_and_update_maze(data, maze, current_position, IR_THRESHOLD, goal_area):
    sensor_dict, current_position = process_sensor_data(data, current_position)
    walls = ir_values_to_walls(sensor_dict, IR_THRESHOLD)
    return maze, walls, current_position, goal_area


def print_maze(maze):
    for row in maze:
        for cell in row:
            if cell == -1:
                print("█", end=" ")
            elif cell == 1000000:
                print(" ", end=" ")
            else:
                print(".", end=" ")
        print()


def send_movements_to_robot(movements, ser):
    for movement in movements:
        ser.write(movement.encode())
        time.sleep(0.5)

def get_goal_area(maze):
    rows, cols = len(maze), len(maze[0])
    goal_area = [
        ((rows - 2) // 2, (cols - 2) // 2),
        ((rows - 2) // 2, (cols + 2) // 2),
        ((rows + 2) // 2, (cols - 2) // 2),
        ((rows + 2) // 2, (cols + 2) // 2)
    ]
    return goal_area

def all_nodes_visited(maze, visited):
    for row in range(len(maze)):
        for col in range(len(maze[0])):
            if maze[row][col] != -1 and (row, col) not in visited:
                return False
    return True

def get_start_position(maze, corner):
    if corner == 'top_left':
        return (0, 0)
    elif corner == 'top_right':
        return (0, len(maze[0]) - 1)
    elif corner == 'bottom_left':
        return (len(maze) - 1, 0)
    elif corner == 'bottom_right':
        return (len(maze) - 1, len(maze[0]) - 1)

def rotate_matrix(matrix):
    transposed = []
    for row in zip(*matrix):
        transposed.append(list(row))
    rotated = []
    for row in transposed:
        rotated.append(row[::-1])
    return rotated

def determineNextMovement(walls, maze, current_position,currentN, heading):
    int_walls = [1 if walls["left"] else 0, 1 if walls["front"] else 0, 1 if walls["right"] else 0]

    pprint(maze)

    newCurrentN = currentN+1

    nextMovement = None

    if(int_walls == [0,0,0]):
        nextMovement =  ["right", "right"]
    if(int_walls == [0,0,1]):
        maze = rotate_matrix(maze)
        if(heading == "north"):
            maze[current_position[0]+1][current_position[1]] = newCurrentN
        elif(heading == "south"):
            maze[current_position[0]-1][current_position[1]] = newCurrentN
        elif(heading == "east"):
            maze[current_position[0]][current_position[1]+1] = newCurrentN
        elif(heading == "west"):
            maze[current_position[0]][current_position[1]-1] = newCurrentN

        nextMovement =  ["right","front"]
    if(int_walls == [0,1,0]):
        if(heading == "north"):
            maze[current_position[0]+1][current_position[1]] = newCurrentN
        elif(heading == "south"):
            maze[current_position[0]-1][current_position[1]] = newCurrentN
        elif(heading == "east"):
            maze[current_position[0]][current_position[1]+1] = newCurrentN
        elif(heading == "west"):
            maze[current_position[0]][current_position[1]-1] = newCurrentN
        nextMovement =  ["front"]
    if(int_walls == [0,1,1]): # Bifurcación
        nextMovement =  [["front"],["right","front"]]
    if(int_walls == [1,0,0]):
        nextMovement =  ["left","front"]
    if(int_walls == [1,0,1]): # Bifurcación
        nextMovement =  ["front"]
    if(int_walls == [1,1,0]): # Bifurcación
        nextMovement =  ["front"]
    if(int_walls == [1,1,1]): # Bifurcación
        nextMovement =  ["front"]

    return nextMovement, maze


def step(ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited, currentN):
    line = None

    # IRL,0,IRD,0,IRF,600,SR1,0,SR2,0

    line = input("Entra: ")

    if(ser != None):
        line = ser.readline().decode().strip()

    print(line)

    data = line.split(",")
    # Leer datos del sensor y actualizar la matriz del laberinto
    maze, walls, current_position, goal_area = read_and_update_maze(data, maze, current_position, IR_THRESHOLD, goal_area)

    print(current_position)

    visited.add(current_position)
    # Verificar si el robot ha llegado a la meta
    if not current_position in goal_area:
        if not all_nodes_visited(maze, visited) and phase == 1:

            nextMovement, maze = determineNextMovement(walls, maze, current_position, currentN, heading)

            print(nextMovement)

            if(all_nodes_visited(maze, visited)):
                phase = 2

        if phase == 2:
            # En la fase 2, envía los movimientos calculados al robot
            # send_movements_to_robot(movements, ser)
            return False,ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited, currentN
    # Mostrar la matriz del laberinto en tiempo real
    print_maze(maze)

    return True,ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited, currentN


def markInMaze(maze,start,currentN):
    maze[start[0]][start[1]]=currentN
    return currentN + 1

def mainWithoutConnection():
    ser = None
    phase = 1
    maze = init_maze(12, 7)

    currentN = 0 # Número que se marcará en el laberinto

    goal_area = get_goal_area(maze)
    start = get_start_position(maze, 'bottom_left')  # Reemplazar

    currentN = markInMaze(maze,start,currentN)

    current_position = start
    heading = 'north'
    IR_THRESHOLD = 500  # Ajustar según la detección de obstáculos de los sensores IR
    visited = set()
    visited.add(start) #// Checkcheckcheck


    while True:
        willContinue,ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited, currentN = step(ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited, currentN)
        if(not willContinue):
            break


def main():
    ser = serial.Serial('COM3', 9600)
    phase = 1
    maze = init_maze(12, 7)

    goal_area = get_goal_area(maze)
    start = get_start_position(maze, 'top_left')  # Reemplazar

    current_position = start
    heading = 'north'
    IR_THRESHOLD = 500  # Ajustar según la detección de obstáculos de los sensores IR
    visited = set()
    visited.add(start) #// Checkcheckcheck

    while True:
        if ser.in_waiting > 0:
            willContinue = step(ser,phase,maze,goal_area,start,current_position,heading,IR_THRESHOLD,visited)
            if(not willContinue):
                break


if __name__ == '__main__':
    mainWithoutConnection()

