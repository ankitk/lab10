
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
from cozmo.objects import LightCube, LightCube1Id


def astar(grid, heuristic):
    goals = grid.getGoals()
    if not goals:
        return
        
    start = grid.getStart()
    route = {}
    route[start] = None
    cost = {}
    cost[start] = 0
    q = PriorityQueue()
    q.put((0, start))
	
    while not q.empty():
        top = q.get()
        current = top[1]
        if current in goals:
            break

        for neighbor in grid.getNeighbors(current):
            current_cost = cost[current] + neighbor[1]
            if neighbor[0] not in cost or current_cost < cost[neighbor[0]]:
                cost[neighbor[0]] = current_cost
                priority = current_cost + heuristic(neighbor[0], goals[0])
                q.put((priority, neighbor[0]))
                route[neighbor[0]] = current
                grid.addVisited(current)
    
    path = []
    i = goals[0]
    path.append(goals[0])
    while route[i] != start:
        path.append(route[i])
        i = route[i]
    
    path.append(start)
    path.reverse()
    grid.setPath(path)

def heuristic(current, goal):
    return math.sqrt(math.pow((goal[1] - current[1]), 2) + math.pow((goal[0] - current[0]), 2))

def cozmo_drive_straight(robot, dist, speed):
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()
    
def setCubePositions(robot, goal, obstacle1, obstacle2):
    if goal != None and obstacle1 != None and obstacle2 != None:
        return (goal, obstacle1, obstacle2)
    
    cubes = []
    for i in robot.world.visible_objects:
        if i != None and isinstance(i, LightCube):
            cubes.append(i)

    for c in cubes:
        if goal == None and c.cube_id == cozmo.objects.LightCube1Id:
            goal = c
        elif obstacle1 == None and c.cube_id == cozmo.objects.LightCube2Id:
            obstacle1 = c
        elif obstacle2 == None and c.cube_id == cozmo.objects.LightCube3Id:
            obstacle2 = c

    print("Goal:%s" % str(goal))
    print("OBS1:%s" % str(obstacle1))
    print("OBS2:%s" % str(obstacle2))
    
    return (goal, obstacle1, obstacle2)

def gotoCenterCubeAndSearch(robot):
        cozmo_drive_straight(robot, 150, 50)
        cozmo_turn_in_place(robot, -90, 10)
        cozmo_drive_straight(robot, 225, 50)
        
        goal = None
        c1 = None
        c2 = None
        
        while goal == None:
            cozmo_turn_in_place(robot, 30, 50)
            (goal, obstacle1, obstacle2) = setCubePositions(robot, goal, c1, c2)
            
        grid.clearStart()
        grid.setStart((13, 9))

def setGoalCoords(robot, grid, goal):
    (xStart, yStart) = grid.getStart()
    (xEnd, yEnd) = goal

    xFactor = robot.pose.position.x
    yFactor = robot.pose.position.y
    xEnd = abs(xEnd - xFactor)/25
    yEnd = abs(yEnd - yFactor)/25
    
    xGoal = int(xStart+xEnd)
    yGoal = int(yStart+yEnd)
    grid.addGoal((xGoal-2, yGoal))

def setObsCoords(robot, grid, obstacle):
    if obstacle == None:
        return

    lock.acquire()
    (xStart, yStart) = grid.getStart()       
    (xCurr, yCurr) = (robot.pose.position.x, robot.pose.position.y)
    (xEnd, yEnd) = (obstacle.pose.position.x, obstacle.pose.position.y)
    
    xEnd = int(abs(xEnd - xCurr)/25)
    yEnd = int(abs(yEnd - yCurr)/25)

    coord0 = (int(xStart+xEnd), int(yStart+yEnd))
    
    coord1 = (int(xStart+xEnd), int(yStart+yEnd)-1)
    coord2 = (int(xStart+xEnd), int(yStart+yEnd)+1)
    
    coord3 = (int(xStart+xEnd)+1, int(yStart+yEnd))
    coord4 = (int(xStart+xEnd)-1, int(yStart+yEnd))
    
    coord5 = (int(xStart+xEnd)+1, int(yStart+yEnd)+1)
    coord6 = (int(xStart+xEnd)-1, int(yStart+yEnd)+1)
    coord7 = (int(xStart+xEnd)+1, int(yStart+yEnd)-1)
    coord8 = (int(xStart+xEnd)-1, int(yStart+yEnd)-1)
    
    coord9 = (int(xStart+xEnd), int(yStart+yEnd)+2)
    coord10 = (int(xStart+xEnd)+1, int(yStart+yEnd)+2)
    coord11 = (int(xStart+xEnd)-1, int(yStart+yEnd)+2)
    
    coord12 = (int(xStart+xEnd), int(yStart+yEnd)-2)
    coord13 = (int(xStart+xEnd)+1, int(yStart+yEnd)-2)
    coord14 = (int(xStart+xEnd)-1, int(yStart+yEnd)-2)

    coords = [coord0, coord1, coord2, coord3, coord4, coord5, coord6, coord7, coord8, coord9, coord10, coord11, coord12, coord3, coord14]
    print("xStart: %i yStart: %i", xStart, yStart)
    print("xEnd: %i yEnd: %i", xEnd, yEnd)
    print(coords)
    grid.addObstacles(coords)
    lock.release()

def driveStraight(robot):
    cozmo_drive_straight(robot, 30, 20)

def moveRight(robot):
    cozmo_turn_in_place(robot, 90, 30)
    driveStraight(robot)
    cozmo_turn_in_place(robot, -90, 30)
    
def moveLeft(robot):
    cozmo_turn_in_place(robot, -90, 30)
    driveStraight(robot)
    cozmo_turn_in_place(robot, 90, 30)

def getStraightLineDistance(path):
    distance = 0
    i = 0
    (startX,startY) = path[0]
    path = path[1:]
    for node in path:
        (x, y) = node
        if startY != y or i > 2:
            return (distance, i)
        
        distance += 22
        i += 1
    
    return (distance, i)
    
def goToNode(robot, srcNode, destNode):
    (srcX, srcY) = srcNode
    (dstX, dstY) = destNode

    if srcX == dstX and dstY > srcY:
        moveRight(robot)
    elif srcX == dstX and dstY < srcY:
        moveLeft(robot)
    elif dstY > srcY:
        driveStraight(robot)
        moveRight(robot)
    elif dstY < srcY:
        driveStraight(robot)
        moveLeft(robot)
        
def cozmoBehavior(robot: cozmo.robot.Robot):
    global grid, stopevent, original
    original = grid.getStart()
    
    robot.move_lift(-3)
    robot.set_head_angle(degrees(-10)).wait_for_completed()
    robot.world._init_light_cubes()
    
    goal = None
    obstacle1 = None
    obstacle2 = None
    (goal, obstacle1, obstacle2) = setCubePositions(robot, goal, obstacle1, obstacle2)
    
    if goal == None:
        gotoCenterCubeAndSearch(robot)
        (goal, obstacle1, obstacle2) = setCubePositions(robot, goal, obstacle1, obstacle2)
    
    goalCoords = (goal.pose.position.x, goal.pose.position.y)
    setGoalCoords(robot, grid, goalCoords)
    currNode = grid.getStart()
    obs1Set = False
    obs2Set = False
    while not stopevent.is_set():
        goals = grid.getGoals()
        if currNode in goals:
            stopevent.set()
            break

        (goal, obstacle1, obstacle2) = setCubePositions(robot, goal, obstacle1, obstacle2)
        if obs1Set == False and obstacle1 != None:
            setObsCoords(robot, grid, obstacle1)
            obs1Set = True 
        if obs2Set == False and obstacle2 != None:
            setObsCoords(robot, grid, obstacle2)
            obs2Set = True
        
        grid.clearPath()
        astar(grid, heuristic)
        path = grid.getPath()
        
        (distance, i) = getStraightLineDistance(path)
        if distance == 0:
            goToNode(robot, currNode, path[1])
            i = 1
        else:
            cozmo_drive_straight(robot, distance, 30)

        currNode = path[i]
        
        grid.clearStart()
        grid.setStart(currNode)
        
        


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent, lock
    lock = threading.Lock()
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

