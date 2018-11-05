# Chingyeu Leekung
# William Zhu

## If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
## try uncommenting the following snippet

try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass

from skimage import color
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
import random
from PIL import Image

from markers import detect, annotator

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *
from cozmo.util import degrees, distance_mm, speed_mmps
from pprint import pprint
from concurrent.futures import CancelledError

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False
converged = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)

def compute_odometry(curr_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


async def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera.
    Since this is an async function, it must be called using await, for example:

        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh)
          (as expected by the particle filter's measurement update)
        - a PIL Image of what Cozmo's camera sees with marker annotations
    '''

    global grid

    # Wait for the latest image from Cozmo
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # Convert the image to grayscale
    image = np.array(image_event.image)
    image = color.rgb2gray(image)

    # Detect the markers
    markers, diag = detect.detect_markers(image, camera_settings, include_diagnostics=True)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x,y,h in marker_list]

    # Annotate the camera image with the markers
    if not show_diagnostic_image:
        annotated_image = image_event.image.resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(annotated_image, markers, scale=2)
    else:
        diag_image = color.gray2rgb(diag['filtered_image'])
        diag_image = Image.fromarray(np.uint8(diag_image * 255)).resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(diag_image, markers, scale=2)
        annotated_image = diag_image

    return marker_list, annotated_image


marker_list = []	# global to be modified by updatePF and used by explore

async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose, marker_list
    global grid, gui, pf, converged

    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)

    ###################

    # YOUR CODE HERE
    print("run")
    goal_mm = (goal[0] * 25.4, goal[1] * 25.4, goal[2])
    goal_pose = cozmo.util.Pose(goal_mm[0], goal_mm[1], 0, angle_z=cozmo.util.Angle(degrees=goal_mm[2]))

    main_loop = asyncio.get_event_loop()
    task = [asyncio.ensure_future(explore(robot)), asyncio.ensure_future(update_particle_filter(robot, camera_settings))]

    try:
        # done, pending = main_loop.run_until_complete(asyncio.wait(task, return_when=asyncio.FIRST_COMPLETED))
        await asyncio.wait(task, return_when=asyncio.FIRST_COMPLETED)
        # await asyncio.gather(asyncio.ensure_future(update_particle_filter(robot, camera_settings)), asyncio.ensure_future(explore(robot)))
        # for task in pending:
        #     task.cancel()
    except CancelledError as e:
        print("Error happened while canceling the task: {e}".format(e=e))
    finally:
        print("Tasks finished")

        time.sleep(2) 
        robot.stop_all_motors()
        m_x, m_y, m_h, m_confident = compute_mean_pose(pf.particles)

        arc = math.degrees(math.atan2(goal[1] - m_y, goal[0] - m_x))

        dif_y = goal[1] - m_y
        dif_x = goal[0] - m_x
        dist = math.sqrt(dif_y**2 + dif_x**2) * 25.4
        turn_angle = diff_heading_deg(arc, m_h)
        await robot.turn_in_place(cozmo.util.degrees(turn_angle - 20)).wait_for_completed()
        await robot.drive_straight(cozmo.util.distance_mm(dist), cozmo.util.speed_mmps(40)).wait_for_completed()
        final_angle = diff_heading_deg(m_h + turn_angle, goal[2])
        await robot.turn_in_place(cozmo.util.degrees(final_angle)).wait_for_completed()

        await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabSurprise).wait_for_completed()



        # pprint("robot pose: " + str(robot.pose))
        # pprint("goal pose: " + str(goal))
        # angle_to_goal = np.arctan( (goal[1] * 25.4 - robot.pose.position.y)/(goal[0] * 25.4 - robot.pose.position.x) ) * 360

        # await robot.turn_in_place(angle_to_goal).wait_for_completed()
        # await robot.drive_straight( ((goal[1] * 25.4)**2 - robot_pose.position.y**2) ** 1/2 ).wait_for_completed

        # await robot.turn_in_place(np.arctan( (goal[1] * 25.4 - robot_pose.position.y)/(goal[0] * 25.4 - robot_pose.position.x) ))

        # await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabSurprise).wait_for_completed()



        # await robot.go_to_pose(goal_pose, relative_to_robot=True).wait_for_completed()
        # pprint("Robot pose: " + str(robot.pose))
        # pprint("Goal pose: " + str(goal_pose))


        # happy_animation(robot)
    ###################

async def update_particle_filter(robot, camera_settings):
    global flag_odom_init, last_pose
    global grid, gui, pf
    global marker_list
    global converged

    while True:
        if robot.is_picked_up:
            print("robot picked up")
            pf = ParticleFilter(grid)
            last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
            converged = False
            time.sleep(2)
            # continue

        # Obtain odometry information
        odom = compute_odometry(robot.pose)
        # order of setting this? not sure
        last_pose = robot.pose
        # pprint(last_pose)

        # Obtain list of currently seen markers and their poses
        marker_list, annotated_image = await marker_processing(robot, camera_settings)

        # Update the particle filter using the information above
        m_x, m_y, m_h, m_confident = pf.update(odom, marker_list)
        # print("Confident: " + str(m_confident))

        # Update the particle filter for the GUI
        gui.show_particles(pf.particles)
        gui.show_mean(m_x, m_y, m_h, m_confident)
        gui.show_camera_image(annotated_image)
        # gui.show_robot(robot)
        gui.updated.set()

        # Determine the robot's action based on the current state of the localization system
        if m_confident:
            converged = True

async def explore(robot):
    global marker_list
    global converged
    while not converged:
        if robot.is_picked_up:
            print("explore, picked up")
            time.sleep(2)
            # continue
        if len(marker_list) >= 1 and marker_list[0][0] > 2.0:
            print("Marker list: " + str(marker_list[0][0] * grid.scale))
            if marker_list[0][0] * grid.scale - 180 < 0:
                await robot.drive_straight(distance_mm(-50), speed_mmps(100)).wait_for_completed()
                await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
                await robot.turn_in_place(degrees(60)).wait_for_completed()
            elif marker_list[0][0] * grid.scale - 200 > 220:
                await robot.drive_straight(distance_mm(50), speed_mmps(100)).wait_for_completed()
                await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
                await robot.turn_in_place(degrees(60)).wait_for_completed()
            else:
                await robot.drive_wheels(-25, 25, duration=0.1)
        else:
            print("marker list is 0")
            await robot.drive_wheels(-25, 25, duration=0.1)




        # await robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
        # print("explore")
        # if robot.is_picked_up:
        #     continue
        # print(str(converged))
        # await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
        # while len(marker_list) == 0:
        #     print("marker list is 0")
        #     await robot.drive_wheels(-25, 25, duration=0.25)	# rotate until you find a marker
        # await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

        # print("Marker list: " + str(marker_list[0][0] * grid.scale))
        # await robot.drive_straight(distance_mm(marker_list[0][0] * grid.scale - 200), speed_mmps(50)).wait_for_completed()	# drive toward marker
        # await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
        
        # await robot.turn_in_place(degrees(45)).wait_for_completed()		# rotate off of marker
    print("explore is done")


class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()
