from machine import Pin, PWM, Timer
import time, machine
import heapq

row, col = 33, 33
start = (31, 1)
end = (row // 2, col // 2)
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

    def move(self, left_dir, right_dir, stop = False, time):
        self.update_speed(stop)
        self.left_motor_forward.value(left_dir[0])
        self.left_motor_backward.value(left_dir[1])
        self.right_motor_forward.value(right_dir[0])
        self.right_motor_backward.value(right_dir[1])
        time.sleep(time)

    def forward(self):
        self.move((1, 0), (1, 0), time)
        
    def forward45(self):
        self.move((1, 0), (1, 0))

    def stop(self):
        self.move((0, 0), (0, 0),True)

    def turnRight(self):
        self.move((0, 1), (1, 0))

    def turnLeft(self):
        self.move((1, 0), (0, 1))

    def turnBack(self):
        self.move((0, 1), (1, 0))

    def setSpeed(self, new_speed):
        self.speed = new_speed
        self.update_speed()
        
enable_pins = [21, 15]
motor_pins = [16, 4, 5, 19]
sensor_ps = [32, 26, 12]

robot = MotorIRControl(enable_pins, motor_pins, sensor_ps, speed = 32767)
sensors = robot.read_sensors()

def map_create():
    row_up = True
    row_down = False
    col_right = False
    col_left = False
    row_pos = row - 2
    col_pos = 1
    FirstTurnRight = 0
    FirstTurnLeft = 0
    stack = [(row_pos, col_pos)]
    visited = set()
    path = []
    steps = 0
    turns = []
    
    maze[row_pos][col_pos] = 0
    
#     while True:
#         if sensors["left"] == 0:
#             row_pos, col_pos = 31, 31
#             break
#         if sensors["right"] == 0:
#             row_pos, col_pos = 31, 1
#             break
#         steps = steps + 1
#         robot.forward()
#     for i in range(steps):
#         visited.add(row_pos - i, col_pos)
    while stack:
        current = stack.pop()
        if current in visited:
            continue

        visited.add(current)
        path.append(current)
        row_pos, col_pos = current

        sensors = robot.read_sensors()
        available_moves = [] # For saving the number of ways to go (eger dongede birden cox gedis yolu olsa yadda saxlamaga)

        if sensors["left"] == 1 and (row_pos, col_pos - 1) not in visited:
            row_up = False
            col_left = True
            row_down = False
            col_right = False
            available_moves.append((col_left, (row_pos, col_pos - 1)))
            FirstTurnLeft = 1
            if FirstTurnLeft == 1 and FirstTurnRight == 0:
                start = (row - 2, col - 2)
                for i in range(steps):
                    maze[row - 2 - i, col - 2] = 0
                FirstTurnLeft =  3

        if sensors["right"] == 1 and (row_pos, col_pos + 1) not in visited:
            row_up = False
            row_down = False
            col_right = True
            col_left = False
            available_moves.append((col_right, (row_pos, col_pos + 1)))
            FirstTurnRight = 1
            if FirstTurnLeft == 0 and FirstTurnRight == 1:
                start = (row - 2, 1)
                for i in range(steps):
                    maze[row - 2 - i, 1] = 0
                FirstTurnRight =  3
                
        if sensors["front"] == 1 and (row_pos - 1, col_pos) not in visited:
            row_up = True
            row_down = False
            col_right = False
            col_left = False
            available_moves.append((row_up, (row_pos - 1, col_pos)))
            # How many steps until finding the first turn
            if FirstTurnLeft == 0 or FirstTurnRight == 0:
                steps = steps + 1
                
        elif sensors["front"] == 0 or len(available_moves) != 1:
            robot.forward45()
            robot.stop()
        
        for move in available_moves:
            stack.append(move[1])

        if available_moves:
            chosen_move = available_moves[0]
            direction, new_pos = chosen_move
            row_pos, col_pos = new_pos
            
            if direction == row_up:
                robot.forward()
                maze[row_pos][col_pos] = 0
            elif direction == col_left:
                robot.turnLeft()
                robot.forward()
                maze[row_pos][col_pos] = 0
                turns.append((row_pos, col_pos + 1))
            elif direction == col_right:
                robot.turnRight()
                robot.forward()
                maze[row_pos][col_pos] = 0
                turns.append((row_pos, col_pos - 1))

        else:
            # Dead end olanda sadece tam deyil bura aid ne fikir olsa deyersiniz. Dead end olanda en son yere qayitma isi ile elaqeli
            if stack:
                robot.turnBack()
                back_pos = stack[-1]
                row_pos, col_pos = back_pos
                robot.forward()

    return maze


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(maze):
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
    