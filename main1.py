from machine import Pin, PWM, Timer
import time, machine
import heapq

row, col = 33, 33
start = (31, 1)
goal = (row // 2, col // 2)
mapCreated = False
PathFinding = False
maze = []
mazeRow = []

# Assigning each cell to 1 and adjusting it if there is any path put 0. The walls will be automatically mapped
for i in range(row):
    for j in range(col):
        mazeRow.append(1)
        
    maze.append(mazeRow)
    mazeRow = []

def right_hand_rule():
    while True:
        sensors = robot.read_sensors()

        if sensors["right"] == 1:
            robot.stop()
            robot.turnRight()
            time.sleep(0.5)
        elif sensors["front"] == 1:
            robot.stop()
            robot.turnLeft()
            time.sleep(0.5)
        else:
            robot.forward()
            time.sleep(0.5)
            


class MotorIRControl():

    def __init__(self, enable_pins, motor_pins, sensor_pins, speed):
        # Motor PWM pins
        self.right_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)
        self.left_motor_enable_pin = PWM(Pin(enable_pins[1]), freq=2000)

        # Motor direction pins
        self.left_motor_forward = Pin(motor_pins[0], Pin.OUT)
        self.left_motor_backward = Pin(motor_pins[1], Pin.OUT)
        self.right_motor_forward = Pin(motor_pins[2], Pin.OUT)
        self.right_motor_backward = Pin(motor_pins[3], Pin.OUT)


        # IR Sensors
        self.front_sensor = Pin(sensor_pins[0], Pin.IN)
        self.left_sensor = Pin(sensor_pins[1], Pin.IN)
        self.right_sensor = Pin(sensor_pins[2], Pin.IN)

        # Default speed
        self.speed = speed
        self.update_speed()

    def read_sensors(self):
        return {
            "front": self.front_sensor.value(),
            "left": self.left_sensor.value(),
            "right": self.right_sensor.value()
        }

    def update_speed(self, stop = False):
        if stop:
            self.right_motor_enable_pin.duty_u16(0)
            self.left_motor_enable_pin.duty_u16(0)
        else:
            self.right_motor_enable_pin.duty_u16(self.speed)
            self.left_motor_enable_pin.duty_u16(self.speed)

    def move(self, left_dir, right_dir, stop = False, time = 0):
        self.update_speed(stop)
        self.left_motor_forward.value(left_dir[0])
        self.left_motor_backward.value(left_dir[1])
        self.right_motor_forward.value(right_dir[0])
        self.right_motor_backward.value(right_dir[1])
        time.sleep(time)

    def forward(self):
        self.move((1, 0), (1, 0), time)
        
    def forward45(self):
        self.move((1, 0), (1, 0), time)

    def stop(self):
        self.move((0, 0), (0, 0),True)

    def turnRight(self):
        self.move((0, 1), (1, 0), time)

    def turnLeft(self):
        self.move((1, 0), (0, 1), time)

    def turnBack(self):
        self.move((0, 1), (1, 0), time)

    def setSpeed(self, new_speed):
        self.speed = new_speed
        self.update_speed()
        
enable_pins = [21, 15]
motor_pins = [16, 4, 5, 19]
sensor_pins = [32, 26, 12]

robot = MotorIRControl(enable_pins, motor_pins, sensor_pins, speed = 32767)
sensors = robot.read_sensors()

UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3

def map_create():
    global start, mapCreated
    row_pos = row - 2
    col_pos = 1
    current_direction = UP
    FirstTurnRight = 0
    FirstTurnLeft = 0
    steps = 0
    stack = [(row_pos, col_pos)]
    visited = set()
    path = []
    turns = []

    maze[row_pos][col_pos] = 0
    
    robot.forward45()
    
    while stack:
        row_pos, col_pos = stack.pop()
        
#         if (row_pos, col_pos) in visited:
#             continue
        
        if maze[row // 2][col // 2] == 0:
            mapCreated = True

        visited.add((row_pos, col_pos))
        path.append((row_pos, col_pos))
        
        sensors = robot.read_sensors()
        available_moves = []
        
        
        # Add available moves based on sensor data and current direction
        if sensors["front"] == 1:
            if current_direction == UP and (row_pos - 1, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos)))
            elif current_direction == RIGHT and (row_pos, col_pos + 1) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1)))
            elif current_direction == DOWN and (row_pos + 1, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos)))
            elif current_direction == LEFT and (row_pos, col_pos - 1) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1)))
            if FirstTurnLeft == 0 and FirstTurnRight == 0:
                steps = steps + 1
            
        if sensors["right"] == 1:
            FirstTurnRight = FirstTurnRight + 1
            if current_direction == UP and (row_pos, col_pos + 1) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1)))
            elif current_direction == RIGHT and (row_pos + 1, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos)))
            elif current_direction == DOWN and (row_pos, col_pos - 1) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1)))
            elif current_direction == LEFT and (row_pos - 1, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos)))
            if FirstTurnRight > 1:
                FirstTurnRight =  3
            if FirstTurnLeft == 0 and FirstTurnRight == 1:
                start = (row - 2, 1)
                for i in range(steps):
                    maze[row - 2 - i][1] = 0
            steps = 0
        
        if sensors["left"] == 1:
            FirstTurnLeft = FirstTurnLeft + 1
            if current_direction == UP and (row_pos, col_pos - 1) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1)))
            elif current_direction == RIGHT and (row_pos - 1, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos)))
            elif current_direction == DOWN and (row_pos, col_pos + 1) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1)))
            elif current_direction == LEFT and (row_pos + 1, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos)))
            if FirstTurnLeft > 1:
                FirstTurnLeft =  3
            if FirstTurnLeft == 1 and FirstTurnRight == 0:
                start = (row - 2, col - 2)
                for i in range(steps):
                    maze[row - 2 - i][col - 2] = 0
            steps = 0

        # Handle movement and direction update
        if available_moves:
            direction, new_pos = available_moves.pop()
            stack.append(new_pos)
            row_posp, col_posp = row_pos, col_pos
            row_pos, col_pos = new_pos

            if direction == UP:
                if current_direction != UP:
                    if current_direction == LEFT:
                        robot.turnRight()
                        turns.append(((row_posp, col_posp), LEFT))
                    elif current_direction == RIGHT:
                        robot.turnLeft()
                        turns.append(((row_posp, col_posp), RIGHT))
                robot.forward()
                current_direction = UP

            elif direction == RIGHT:
                if current_direction != RIGHT:
                    if current_direction == UP:
                        robot.turnRight()
                        turns.append(((row_posp, col_posp), UP))
                    elif current_direction == DOWN:
                        robot.turnLeft()
                        turns.append(((row_posp, col_posp), DOWN))
                robot.forward()
                current_direction = RIGHT

            elif direction == DOWN:
                if current_direction != DOWN:
                    if current_direction == RIGHT:
                        robot.turnRight()
                        turns.append(((row_posp, col_posp), RIGHT))
                    elif current_direction == LEFT:
                        robot.turnLeft()
                        turns.append(((row_posp, col_posp), LEFT))
                robot.forward()
                current_direction = DOWN

            elif direction == LEFT:
                if current_direction != LEFT:
                    if current_direction == UP:
                        robot.turnLeft()
                        turns.append(((row_posp, col_posp), UP))
                    elif current_direction == DOWN:
                        robot.turnRight()
                        turns.append(((row_posp, col_posp), DOWN))
                robot.forward()
                current_direction = LEFT
                
            maze[row_pos][col_pos] = 0

        else:
            # Backtracking when no moves are available
            if turns:
                back_pos, direction = turns[-1]
                if back_pos[0] < row_pos:
                    while current_direction != UP:
                        robot.turnRight()
                        current_direction = (current_direction + 1) % 4
                elif back_pos[0] > row_pos:
                    while current_direction != DOWN:
                        robot.turnRight()
                        current_direction = (current_direction + 1) % 4
                elif back_pos[1] < col_pos:
                    while current_direction != LEFT:
                        robot.turnRight()
                        current_direction = (current_direction + 1) % 4
                elif back_pos[1] > col_pos:
                    while current_direction != RIGHT:
                        robot.turnRight()
                        current_direction = (current_direction + 1) % 4
                        
                steps = abs(back_pos[0] - row_pos) + abs(back_pos[1] - col_pos)
                row_pos, col_pos = back_pos
                stack.append((row_pos, col_pos))
                for i in range(steps):
                    robot.forward()
                robot.stop()
                if direction == UP:
                    if current_direction == LEFT:
                        robot.turnRight()
                    elif current_direction == RIGHT:
                        robot.turnLeft()
                if direction == DOWN:
                    if current_direction == LEFT:
                        robot.turnLeft()
                    elif current_direction == RIGHT:
                        robot.turnRight()
                if direction == LEFT:
                    if current_direction == UP:
                        robot.turnRight()
                    elif current_direction == DOWN:
                        robot.turnLeft()
                if direction == RIGHT:
                    if current_direction == UP:
                        robot.turnLeft()
                    elif current_direction == DOWN:
                        robot.turnRight()
                current_direction = direction
                        
            else:
                mapCreated = True
                break

    return maze

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(start, goal, maze):
    DIRECTIONS = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    open_list = []
    closed_list = set()
    came_from = {}
    
    # G score (cost from start to a node)
    g_score = {start: 0}
    
    # F score (estimated total cost)
    f_score = {start: heuristic(start, goal)}
    
    heapq.heappush(open_list, (f_score[start], start))
    
    while open_list:
        _, current = heapq.heappop(open_list)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        closed_list.add(current)
        
        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            
            if 0 <= neighbor[0] < len(maze) and 0 <= neighbor[1] < len(maze[0]) and maze[neighbor[0]][neighbor[1]] == 0:
                if neighbor in closed_list:
                    continue
                
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    
    return None

def path_find(start, goal, maze):
    path = []
    direction = []
    current_direction = UP
    row = 0
    rowNext = 0
    col = 0
    colNext = 0
    path = astar(start, goal, maze)
    for i in range(len(path) - 1):
        row = path[i][0]
        rowNext = path[i + 1][0]
        col = path[i][1]
        colNext = path[i + 1][1]
        if col > colNext and current_direction != LEFT:
            if current_direction == UP:
                direction.append([2, 90])
                direction.append([0, 9])
            elif current_direction == DOWN:
                direction.append([1, 90])
                direction.append([0, 9])
            current_direction = LEFT
        elif col < colNext and current_direction != RIGHT:
            if current_direction == UP:
                direction.append([1, 90])
                direction.append([0, 9])
            elif current_direction == DOWN:
                direction.append([2, 90])
                direction.append([0, 9])
            current_direction = RIGHT
        elif row > rowNext and current_direction != UP:
            if current_direction == LEFT:
                direction.append([1, 90])
                direction.append([0, 9])
            elif current_direction == RIGHT:
                direction.append([2, 90])
                direction.append([0, 9])
            current_direction = UP
        elif row < rowNext and current_direction != DOWN:
            if current_direction == LEFT:
                direction.append([2, 90])
                direction.append([0, 9])
            elif current_direction == RIGHT:
                direction.append([1, 90])
                direction.append([0, 9])
            current_direction = DOWN
        else:
            direction.append([0, 9])
            
    return direction

def main():
    pathDirection = path_find(start, goal, maze)
    robot.forward45()
    for i in range(len(pathDirection)):
        if pathDirection[i][0] == 0:
            robot.forward()
        elif pathDirection[i][0] == 1:
            robot.turnRight()
        else:
            robot.turnLeft()
    PathFinding = True