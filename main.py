from machine import Pin, PWM, I2C
from vl53l0x import VL53L0X
import time, machine
import heapq

row, col = 33, 33
start = (31, 1)
goal = (row // 2, col // 2)
mapCreated = False
PathFinding = False
maze = []
mazeRow = []

i2c_left = I2C(0, scl=machine.Pin(27), sda=machine.Pin(33))
i2c_right = I2C(1, scl=machine.Pin(26), sda=machine.Pin(32))

sensor_front = Pin(14, Pin.IN)
sensor_left = VL53L0X(i2c_left)
sensor_right = VL53L0X(i2c_right)

main_switch = Pin(, Pin.IN, Pin.PULL_DOWN)
map_switch = Pin(, Pin.IN, Pin.PULL_DOWN)
button = Pin(, Pin.IN, Pin.PULL_DOWN)

# Assigning each cell to 1 and adjusting it if there is any path put 0. The walls will be automatically mapped
for i in range(row):
    for j in range(col):
        mazeRow.append(1)
        
    maze.append(mazeRow)
    mazeRow = []

class MotorIRControl():

    def __init__(self, enable_pins, motor_pins, speed):
        # Motor PWM pins
        self.right_motor_enable_pin = PWM(Pin(enable_pins[1]), freq=2000)
        self.left_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)

        # Motor direction pins
        self.left_motor_forward = Pin(motor_pins[0], Pin.OUT)
        self.left_motor_backward = Pin(motor_pins[1], Pin.OUT)
        self.right_motor_forward = Pin(motor_pins[2], Pin.OUT)
        self.right_motor_backward = Pin(motor_pins[3], Pin.OUT)

        # Default speed
        self.speed = speed
        self.update_speed()
        
        self.pwm_per_cm = pwm

    def update_speed(self, stop = False):
        if stop:
            self.right_motor_enable_pin.duty_u16(0)
            self.left_motor_enable_pin.duty_u16(0)
        else:
            self.right_motor_enable_pin.duty_u16(self.speed)
            self.left_motor_enable_pin.duty_u16(self.speed)

    def move(self, left_dir, right_dir, stop = False, duration = 0):
        self.update_speed(stop)
        self.left_motor_forward.value(left_dir[0])
        self.left_motor_backward.value(left_dir[1])
        self.right_motor_forward.value(right_dir[0])
        self.right_motor_backward.value(right_dir[1])
        time.sleep(duration)
        self.left_motor_forward.value(0)
        self.left_motor_backward.value(0)
        self.right_motor_forward.value(0)
        self.right_motor_backward.value(0)
        time.sleep(duration)

    def forward(self):
        left_val = sensor_left.range  # mm
        time.sleep(0.1)
        right_val = sensor_right.range  # mm
        time.sleep(0.1)

        diff = (left_val - right_val) / 10  # mm to cm
        pwm_adjust = int(abs(diff) * self.pwm_per_cm)
        
        pwm_left = self.speed
        pwm_right = self.speed

        if diff != 0:
            if diff > 0:
                # Saga yaxindir, solun suretini azalt
                pwm_left = max(0, self.speed - pwm_adjust)
            elif diff < 0:
                # Sola yaxindir, sagin suretini azalt
                pwm_right = max(0, self.speed - pwm_adjust)
            
            self.left_motor_enable_pin.duty_u16(pwm_left)
            self.right_motor_enable_pin.duty_u16(pwm_right)
            self.left_motor_forward.value(1)
            self.left_motor_backward.value(0)
            self.right_motor_forward.value(1)
            self.right_motor_backward.value(0)
            time.sleep(duration)
            self.left_motor_forward.value(0)
            self.left_motor_backward.value(0)
            self.right_motor_forward.value(0)
            self.right_motor_backward.value(0)
            time.sleep(duration)
        else:
            self.move((1, 0), (1, 0), duration)
        
        self.update_speed()
        
    def forward45(self):
        self.move((1, 0), (1, 0), duration)

    def stop(self):
        self.move((0, 0), (0, 0),True)

    def turnRight(self):
        self.move((1, 0), (0, 1), duration)

    def turnLeft(self):
        self.move((0, 1), (1, 0), duration)

    def turnBack(self):
        self.move((1, 0), (0, 1), duration)

    def setSpeed(self, new_speed):
        self.speed = new_speed
        self.update_speed()

def right_hand_rule():
    while True:
        sensor_front_range = sensor_front.value()
        time.sleep(0.1)
        sensor_left_range = sensor_left.range # mm
        time.sleep(0.1)
        sensor_right_range = sensor_right.range # mm
        time.sleep(0.1)
        
        sensors = [sensor_front_range, sensor_left_range, sensor_right_range]
        
        if sensors[2] >= 23:
            robot.turnRight()
            robot.forward()
            robot.forward()
        elif sensors[1] >= 23:
            robot.turnLeft()
            robot.forward()
            robot.forward()
        else:
            robot.forward()
            robot.forward()
        
enable_pins = [2, 19] # PWMA, PWMB
motor_pins = [1, 5, 18, 21] # AIN1, AIN2, BIN1, BIN2

robot = MotorIRControl(enable_pins, motor_pins, speed = 32767)

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
        
        if maze[row // 2][col // 2] == 0:
            mapCreated = True
        
        if map_switch.value() == 0:
            mapCreated = True
            break

        visited.add((row_pos, col_pos))
        
        sensor_front_range = sensor_front.value()
        time.sleep(0.1)
        sensor_left_range = sensor_left.range # mm
        time.sleep(0.1)
        sensor_right_range = sensor_right.range # mm
        time.sleep(0.1)
        
        sensors = [sensor_front_range, sensor_left_range, sensor_right_range]
        available_moves = []
        
        
        # Add available moves based on sensor data and current direction
        if sensors[2] >= 23:
            FirstTurnRight = FirstTurnRight + 1
            if current_direction == UP and (row_pos, col_pos + 1) not in visited and (row_pos, col_pos + 2) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1), (row_pos, col_pos + 2)))
                visited.add((row_pos, col_pos + 1))
            elif current_direction == RIGHT and (row_pos + 1, col_pos) not in visited and (row_pos + 2, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos), (row_pos + 2, col_pos)))
                visited.add((row_pos + 1, col_pos))
            elif current_direction == DOWN and (row_pos, col_pos - 1) not in visited and (row_pos, col_pos - 2) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1), (row_pos, col_pos - 2)))
                visited.add((row_pos, col_pos - 1))
            elif current_direction == LEFT and (row_pos - 1, col_pos) not in visited and (row_pos - 2, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos), (row_pos - 2, col_pos)))
                visited.add((row_pos - 1, col_pos))
            if FirstTurnRight > 1:
                FirstTurnRight =  3
            if FirstTurnLeft == 0 and FirstTurnRight == 1:
                start = (row - 2, 1)
            steps = 0
        
        if sensors[1] >= 23:
            FirstTurnLeft = FirstTurnLeft + 1
            if current_direction == UP and (row_pos, col_pos - 1) not in visited and (row_pos, col_pos - 2) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1), (row_pos, col_pos - 2)))
                visited.add((row_pos, col_pos - 1))
            elif current_direction == RIGHT and (row_pos - 1, col_pos) not in visited and (row_pos - 2, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos), (row_pos - 2, col_pos)))
                visited.add((row_pos - 1, col_pos))
            elif current_direction == DOWN and (row_pos, col_pos + 1) not in visited and (row_pos, col_pos + 2) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1), (row_pos, col_pos + 2)))
                visited.add((row_pos, col_pos + 1))
            elif current_direction == LEFT and (row_pos + 1, col_pos) not in visited and (row_pos + 2, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos), (row_pos + 2, col_pos)))
                visited.add((row_pos + 1, col_pos))
            if FirstTurnLeft > 1:
                FirstTurnLeft =  3
            if FirstTurnLeft == 1 and FirstTurnRight == 0:
                start = (row - 2, col - 2)
                for i in range(steps):
                    maze[row - 2 - i][col - 2] = 0
                    maze[row - 2 - i][1] = 1
            steps = 0
        
        if sensors[0] == 0:
            if current_direction == UP and (row_pos - 1, col_pos) not in visited and (row_pos - 2, col_pos) not in visited:
                available_moves.append((UP, (row_pos - 1, col_pos), (row_pos - 2, col_pos)))
                visited.add((row_pos - 1, col_pos))
            elif current_direction == RIGHT and (row_pos, col_pos + 1) not in visited and (row_pos, col_pos + 2) not in visited:
                available_moves.append((RIGHT, (row_pos, col_pos + 1), (row_pos, col_pos + 2)))
                visited.add((row_pos, col_pos + 1))
            elif current_direction == DOWN and (row_pos + 1, col_pos) not in visited and (row_pos + 2, col_pos) not in visited:
                available_moves.append((DOWN, (row_pos + 1, col_pos), (row_pos + 2, col_pos)))
                visited.add((row_pos + 1, col_pos))
            elif current_direction == LEFT and (row_pos, col_pos - 1) not in visited and (row_pos, col_pos - 2) not in visited:
                available_moves.append((LEFT, (row_pos, col_pos - 1), (row_pos, col_pos - 2)))
                visited.add((row_pos, col_pos - 1))
            if FirstTurnLeft == 0 and FirstTurnRight == 0:
                steps = steps + 2

        # Handle movement and direction update
        if available_moves:
            direction, skip_pos, new_pos = available_moves[0]
            stack.append(new_pos)
            row_posp, col_posp = row_pos, col_pos
            row_poss, col_poss = skip_pos
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
                robot.forward()
                current_direction = LEFT
                
            maze[row_poss][col_poss] = 0
            maze[row_pos][col_pos] = 0

        else:
            # Backtracking when no moves are available
            if turns:
                back_pos, direction = turns.pop()
                if back_pos[1] < col_pos:
                    if current_direction == RIGHT:
                        robot.turnBack()
                    elif current_direction == UP:
                        robot.turnLeft()
                    elif current_direction == DOWN:
                        robot.turnRight()
                    current_direction = LEFT
                elif back_pos[1] > col_pos:
                    if current_direction == LEFT:
                        robot.turnBack()
                    elif current_direction == UP:
                        robot.turnRight()
                    elif current_direction == DOWN:
                        robot.turnLeft()
                    current_direction = RIGHT
                elif back_pos[0] < row_pos:
                    if current_direction == RIGHT:
                        robot.turnLeft()
                    elif current_direction == LEFT:
                        robot.turnRight()
                    elif current_direction == DOWN:
                        robot.turnBack()
                    current_direction = UP
                elif back_pos[0] > row_pos:
                    if current_direction == RIGHT:
                        robot.turnRight()
                    elif current_direction == LEFT:
                        robot.turnLeft()
                    elif current_direction == UP:
                        robot.turnBack()
                    current_direction = DOWN
                        
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
                        robot.turnLeft()
                    elif current_direction == DOWN:
                        robot.turnRight()
                if direction == RIGHT:
                    if current_direction == UP:
                        robot.turnRight()
                    elif current_direction == DOWN:
                        robot.turnLeft()
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
    for i in range(0, len(path) - 1):
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
    global PathFinding
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

while True:
    if button.value() == 1:
        time.sleep(5)
        right_hand_rule()
    elif map_swtich_pin.value() == 1 and mapCreated == False:
        time.sleep(5)
        map_create()
    elif main_switch_pin.value() == 1 and mapCreated == True and PathFinding == False:
        time.sleep(5)
        main()

maze = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]