import cozmo
import math
import sys
import time

from cmap import *
from gui import *
from utils import *

import random

MAX_NODES = 20000
INCH_TO_MM = 25.4
goal_center_found = False
offset_angle = 0
cur_angle = 0

def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    dist = get_dist(node0, node1)
    if dist < limit:
        return node1
    difference = ((node1.x - node0.x), (node1.y - node0.y))
    diffAngle = np.arctan2(difference[1], difference[0])
    new_node = Node((node0.x + (np.cos(diffAngle) * limit), node0.y + (np.sin(diffAngle) * limit)))
    return new_node

    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    pass
    ############################################################################
    probOfGoal = random.randint(0, 101)
    if probOfGoal < 6 and len(cmap.get_goals()) > 0:
        goal_node = cmap.get_goals()[0]
        rand_node = Node((goal_node.x, goal_node.y))
    else:
        width = random.randint(0, cmap.width)
        height = random.randint(0, cmap.height)
        rand_node = Node((width, height))
        while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
            width = random.randint(0, cmap.width)
            height = random.randint(0, cmap.height)
            rand_node = Node((width, height))
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        minDist = 100000
        for node in cmap.get_nodes():
            dist = get_dist(node, rand_node)
            if dist < minDist:
                nearest_node = node
                minDist = dist
        limited_node = step_from_to(nearest_node, rand_node)
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, limited_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    global offset_angle, cur_angle

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    marked = {}
    start_node = Node((6 * INCH_TO_MM, 10 * INCH_TO_MM))
    cmap.set_start(start_node)
    center_node = Node((12 * INCH_TO_MM, 9 * INCH_TO_MM))
    cmap.add_goal(center_node)
    RRT(cmap, cmap.get_start())
    while True:
        path = cmap.get_smooth_path()
        update_cmap, goal_center, start_pos, goal_reached = await move_robot_on_path(robot, marked, path)
        if update_cmap:
            cmap.reset()
            cmap.set_start(start_pos)
            RRT(cmap, cmap.get_start())
        elif goal_reached:
            break
        else:
            # Turn around a bunch
            print("Should look for goal")
            await look_for_goal(robot, marked, center_node)
            # await robot.turn_in_place(cozmo.util.degrees(offset_angle)).wait_for_completed()
            print("Offset angle: " + str(offset_angle))
            cmap.reset()
            cmap.set_start(center_node)
            RRT(cmap, cmap.get_start())
            

async def look_for_goal(robot, marked, cur_pos):
    global goal_center_found
    global offset_angle, cur_angle

    offset_angle = 0
    rotate_angle = 30
    while not goal_center_found:
        update_cmap, goal_center = await detect_cube_and_update_cmap(robot, marked, cur_pos)

        await robot.turn_in_place(cozmo.util.degrees(rotate_angle)).wait_for_completed()
        cur_angle = get_angle(cur_angle + rotate_angle)
        offset_angle -= rotate_angle
        if goal_center is not None:
            goal_center_found = True
    

async def move_robot_on_path(robot, marked, path):
    global goal_center_found

    cur_node = path[0]
    first_node = True
    # cur_angle = 0
    for node in path:
        if first_node:
            first_node = False
            continue
        difference = ((node.x - cur_node.x), (node.y - cur_node.y))
        diff_angle = np.arctan2(difference[1], difference[0])
        turn_angle = diff_heading_deg(math.degrees(diff_angle), cur_angle)
        await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()
        cur_angle = get_angle(cur_angle + turn_angle)
        update_cmap, goal_center = await detect_cube_and_update_cmap(robot, marked, cur_node)
        if update_cmap:
            if goal_center is not None:
                goal_center_found = True
            turn_angle = diff_heading_deg(0, cur_angle)
            await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()
            cur_angle = get_angle(cur_angle + turn_angle)
            return update_cmap, goal_center, cur_node, False

        dist = get_dist(node, cur_node)
        await robot.drive_straight(cozmo.util.distance_mm(dist), cozmo.util.speed_mmps(40)).wait_for_completed()
        cur_node = node

    goal_reached = goal_center_found
    return False, None, None, goal_reached

def get_angle(angle):
    if angle > 360:
        angle -= 360
    elif angle < 0:
        angle += 360
    return angle

def diff_heading_deg(heading1, heading2):
	dh = heading1 - heading2
	while dh > 180:
		dh -= 360
	while dh <= -180:
		dh += 360
	return dh

def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    cos = math.cos(local_angle)
    sin = math.sin(local_angle)
    newX = node.x * cos - node.x * sin
    newY = node.y * sin + node.y * cos
    new_node = Node((newX + local_origin.x, newY + local_origin.y))
    return new_node


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        print(str(robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id))
        print(str(obj.object_id))
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:
            print("Goal Cube")
            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map3.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()