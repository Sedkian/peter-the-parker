import heapq
from random import randint
import random

def heuristic(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal):
    """Find the shortest path from start to goal using A* algorithm."""
    # Directions for moving in the grid (up, down, left, right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    # Priority queue for open nodes: (f_score, (x, y))
    open_queue = []
    heapq.heappush(open_queue, (0, start))
    
    # Dictionaries to store g_scores and f_scores
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, goal)}
    
    # Dictionary to store the path
    came_from = {}
    
    while open_queue:
        _, current = heapq.heappop(open_queue)
        
        # If we reach the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path
        
        # Explore neighbors
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if the neighbor is within the grid and not an obstacle
            if (0 <= neighbor[0] < len(grid) and 
                0 <= neighbor[1] < len(grid[0]) and 
                grid[neighbor[0]][neighbor[1]] == 0):
                
                # Calculate tentative g_score
                tentative_g_score = g_scores[current] + 1
                
                if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                    # Update the path and scores
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g_score
                    f_scores[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_queue, (f_scores[neighbor], neighbor))
    
    # If no path is found
    return None

def generate_random_grid(row, column, weight_lot = 1, weight_clear = 1):
    grid = []
    for i in range(row):
        grid.append(random.choices([0,1], weights=[weight_clear,weight_lot], k=column))
    return grid

def visualize_grid(grid, path = None, start = None, end = None):
    for i in range(len(grid[0])+1):
        if i == 0:
            print("+", end="")
        else:
            print("--", end="")
    print("-+", end="")
    print()

    if path == None:
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if j == 0:
                    print("|", end=" ")
                if grid[i][j] == 1:
                    print("X", end=" ")  # Obstacle
                elif (i,j) == start:
                    print("S", end=" ")  # Start
                elif (i,j) == end:
                    print("E", end=" ")  # End
                else:
                    print(".", end=" ")  # Empty space
            print("|", end=" ")
            print()

    else:
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if j == 0:
                    print("|", end=" ")
                if (i, j) in path:
                    print("P", end=" ")  # Path
                elif grid[i][j] == 1:
                    print("X", end=" ")  # Obstacle
                elif (i,j) == start:
                    print("S", end=" ")  # Start
                elif (i,j) == end:
                    print("E", end=" ")  # End
                else:
                    print(".", end=" ")  # Empty space
            print("|", end=" ")
            print()

    for i in range(len(grid[0])+1):
        if i == 0:
            print("+", end="")
        else:
            print("--", end="")
    print("-+", end="")
    print()

def calcuate_moves(path):
    if path is None:
        return None 
    movements = []
    curr = "east"
    length = 0
    start_block = path[0]
    for i in range(len(path)-1):
        curr_block = path[i]
        next_block = path[i+1]
        H = curr_block[0] - next_block[0]
        V = curr_block[1] - next_block[1]
        match [H,V]:
            case [0,1]:
                d = "west"
            case [0,-1]:
                d = "east"
            case [1,0]:
                d = "north"
            case [-1,0]:
                d = "south"
        if(curr != d):
            #For rotations: 
            # CounterClockWise 90 = 0
            # ClockWise 90 = 1
            # ClockWise 180 = 2
            match[curr,d]:
                case["north","east"]:
                    rot = 1
                case["north","west"]:
                    rot = 0
                case["north","south"]:
                    rot = 2
                case["east","north"]:
                    rot = 0
                case["east","south"]:
                    rot = 1
                case["east","west"]:
                    rot = 2
                case["south","north"]:
                    rot = 2
                case["south","west"]:
                    rot = 1
                case["south","east"]:
                    rot = 0
                case["west","north"]:
                    rot = 1
                case["west","east"]:
                    rot = 2
                case["west","south"]:
                    rot = 0 
            print("Travel line of length {0} from Blocks {1}-{2}".format(length, start_block,curr_block))
            print("Rotation: {0}".format(rot))  
            movements.append([0, length]) #transitional movement
            movements.append([1,rot]) #rotational movement
            curr = d
            start_block = curr_block
            length = 1
        else:
            length += 1
    print("Last line:\nTravel line of length {0} from Blocks {1}-{2}".format(length, start_block,curr_block))
    movements.append([0, length]) #transitional movement
        
    return movements
            
def moves_to_hex(moves):
    output_string = "FF 55 " + str.format("{:x} ",len(moves))
    temp_string = ""
    for mo in moves:
        print(mo)
        match(mo[0]):
            case 0: #translation
                temp_string = mo[1] << 1 
            case 1: #rotation
                temp_string = (mo[1] << 1) + 1 
        output_string += str(temp_string) + " "
    return output_string

def endpoints_open(grid, start, end):
    grid[start[0]][start[1]] = 0
    grid[end[0]][end[1]] = 0

grid = [
    [1, 0, 1, 1, 0, 0],
    [1, 0, 1, 1, 0, 1],
    [1, 0, 1, 1, 0, 1],
    [1, 0, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 1],
]
s = (4, 0)  #(row, column)
g = (0, 5) 

grid2 = [
    [1, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 1],
    [1, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 1],
]
s2 = (4,0)
g2 = (2,2)

grid3 = [
    [1, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 0, 1],
    [1, 0, 0, 1, 0, 1],
    [1, 1, 0, 1, 0, 1],
    [0, 0, 0, 1, 0, 1],
]
s3 = (4,0)
g3 = (4,4)


#rid = generate_random_grid(10,10,1,3)
visualize_grid(grid, None, None, None)
endpoints_open(grid, s,g)
visualize_grid(grid, None, s, g)
# Find the path
#path = a_star(grid, s, g)

# Visualize the grid with the path
#visualize_grid(grid, s, g, None)
#visualize_grid(grid, s, g, path)

#moves = calcuate_moves(path)
#gw = generate_random_grid(10,10, 5)
#visualize_grid(grid, None, None, None)
#visualize_grid(grid, path, s, g)
#bluetooth_input = moves_to_hex(moves)