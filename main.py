from vpython import box, sphere, vector, rate, color, canvas
import heapq
import random


ROWS, COLS = 12, 12
CELL_SIZE = 2

scene = canvas(title="3D Robot Path Planning",
               width=900, height=700,
               background=color.gray(0.8))


scene.forward = vector(-1, -0.6, -1)  
scene.up = vector(0, 1, 0)            


center_x = (ROWS - 1) * CELL_SIZE / 2
center_z = (COLS - 1) * CELL_SIZE / 2
scene.center = vector(center_x, 0, center_z)


grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

for _ in range(35):
    r = random.randint(0, ROWS - 1)
    c = random.randint(0, COLS - 1)
    grid[r][c] = 1


start = (0, 0)
goal = (ROWS - 1, COLS - 1)
grid[start[0]][start[1]] = 0
grid[goal[0]][goal[1]] = 0

ground = box(pos=vector((ROWS-1), -0.1, (COLS-1)),
             size=vector(ROWS * CELL_SIZE, 0.2, COLS * CELL_SIZE),
             color=color.white)


for r in range(ROWS):
    for c in range(COLS):
        if grid[r][c] == 1:
            box(pos=vector(r*CELL_SIZE, 1, c*CELL_SIZE),
                size=vector(CELL_SIZE, CELL_SIZE, CELL_SIZE),
                color=color.green)


robot_body = box(pos=vector(start[0]*CELL_SIZE, 1, start[1]*CELL_SIZE),
                 size=vector(CELL_SIZE, CELL_SIZE, CELL_SIZE),
                 color=color.red)

robot_head = sphere(pos=robot_body.pos + vector(0, CELL_SIZE/1.2, 0),
                    radius=0.6,
                    color=color.white)


left_arm = box(pos=robot_body.pos + vector(-CELL_SIZE/1.2, 0, 0),
               size=vector(0.4, CELL_SIZE*0.8, 0.4),
               color=color.red)

right_arm = box(pos=robot_body.pos + vector(CELL_SIZE/1.2, 0, 0),
                size=vector(0.4, CELL_SIZE*0.8, 0.4),
                color=color.red)


goal_cube = box(pos=vector(goal[0]*CELL_SIZE, 1, goal[1]*CELL_SIZE),
                size=vector(CELL_SIZE, CELL_SIZE, CELL_SIZE),
                color=color.yellow)



def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal):
    moves = [(0,1),(0,-1),(1,0),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        close_set.add(current)
        for dx, dy in moves:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                new_g = gscore[current]+1
                if neighbor in close_set and new_g >= gscore.get(neighbor, 0):
                    continue
                if new_g < gscore.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    gscore[neighbor] = new_g
                    fscore[neighbor] = new_g + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []


path = astar(start, goal)


for step in path:
    target = vector(step[0]*CELL_SIZE, 1, step[1]*CELL_SIZE)
    
    while (robot_body.pos - target).mag > 0.05:
        rate(20)  
        direction = (target - robot_body.pos) * 0.2
        robot_body.pos += direction
        robot_head.pos = robot_body.pos + vector(0, CELL_SIZE/1.2, 0)

        
        left_arm.pos = robot_body.pos + vector(-CELL_SIZE/1.2, 0, 0)
        right_arm.pos = robot_body.pos + vector(CELL_SIZE/1.2, 0, 0)


while True:
    rate(10)