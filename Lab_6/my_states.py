import sys
import time
import cozmo
import cozmo.behavior
import classifyimage
import time

from markers import detect, annotator
from skimage import color
from cozmo.util import degrees, distance_mm, speed_mmps
from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from state import State
from setting import *
from particle_filter import *
from PIL import Image
from localize import *
from classifyimage import *

############## State Machine ##############
class LocalizeState(State):
    """
        Localize, only one time?
    """
    stateName = ''
    """
    robot: Cozmo robot
    cube: character for what cube ('A', 'B', 'C', 'D')
    """
    def __init__(self, robot, cube):
        self.robot = robot
        self.cube = cube
        print ('Current state: {} Cube: {}'.format(str(self), cube))
        """
        Localize
        """ 

    def go_to_next_state(self):
        return PickUpPlanState(self.robot, self.cube)

    async def do_action(self):
        # show pf gui?
        # init
        print("Localize action")
        # await self.robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

        # if SHOW_PF_GUI:
        #     gui.show_particles(pf.particles)
        #     gui.show_mean(0, 0, 0)
        #     gui.start()

        # Obtain the camera intrinsics matrix
        fx, fy = self.robot.camera.config.focal_length.x_y
        cx, cy = self.robot.camera.config.center.x_y
        camera_settings = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float)

        await self.robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

        task = [asyncio.ensure_future(explore(self.robot)), asyncio.ensure_future(update_particle_filter(self.robot, camera_settings))]
        try:
            await asyncio.wait(task, return_when=asyncio.FIRST_COMPLETED)
        except CancelledError as e:
            print("Error happened while canceling the task: {e}".format(e=e))
        finally:
            print("Particle filter converged")
            await self.robot.say_text("Particle filter converged", duration_scalar=0.5).wait_for_completed()


class ClassifyImageState(State):
    """
        Go through each of A, B, C, D locations, in that order?
        Maybe keep a global variable to keep track of which locations we've visited
    """

    def __init__(self, robot, cube):
        self.robot = robot
        self.cube = cube
        print ('Current state: {} Cube: {}'.format(str(self), cube))
        """
        Plan route to cube
        """

    async def do_action(self):
        # TODO
        # show pf gui?
        # init
        print("Classify Images")
        await self.robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

        marker_positions = [(10.5 * INCH_TO_MM, 0), (650 - 7.5 * INCH_TO_MM, 0), (0, 450 - 8.5 * INCH_TO_MM), (650 - 11.5 * INCH_TO_MM, 450), (650, 450 - 5.5 * INCH_TO_MM), (650, 6.5 * INCH_TO_MM)]
        
        goal_marker_list = []
        remaining_markers = ["drone", "insepction", "plane", "order"]

        start_node = compute_mean_pose(pf.particles)
        off_center_node = (9 * INCH_TO_MM, 9 * INCH_TO_MM)
        for marker in marker_positions:
            cmap.set_start(start_node)
            cmap.add_goal(marker)
            RRT(cmap, cmap.get_start())
            while True:
                path = cmap.get_smooth_path()
                update_cmap, goal_center, start_pos, goal_reached = await move_robot_on_path(robot, marked, path)
                if update_cmap:
                    cmap.reset()
                    cmap.set_start(start_pos)
                    RRT(cmap, cmap.get_start())
                elif goal_reached:
                    marker_list, annotated_image = await marker_processing(robot, camera_settings)
                    imageName = classify_image(annotated_image)
                    if imageName is in remaining_markers:
                        goal_marker_list.append((imageName, marker))
                        remaining_markers.remove(imageName)
                    break
                else:
                    # Turn around a bunch
                    print("Should look for goal")
                    await look_for_goal(robot, marked, off_center_node)
                    cmap.reset()
                    cmap.set_start(center_node)
                    RRT(cmap, cmap.get_start())
            cmap.reset()
            start_node = marker


    def go_to_next_state(self):
        return PickUpCubeState(self.robot, self.cube)

class PickUpPlanState(State):
    """
        Go through each of A, B, C, D locations, in that order?
        Maybe keep a global variable to keep track of which locations we've visited
    """

    def __init__(self, robot, cube):
        self.robot = robot
        self.cube = cube
        print ('Current state: {} Cube: {}'.format(str(self), cube))
        """
        Plan route to cube
        """

    async def do_action(self):
        # TODO
        pass

    def go_to_next_state(self):
        return PickUpCubeState(self.robot, self.cube)

class PickUpCubeState(State):
    """
        Picks up a cube, pass in which cube location (i.e. A, B, C ,D)
    """
    def __init__(self, robot, cube):
        self.robot = robot
        self.cube = cube
        print ('Current state: {} Cube: {}'.format(str(self), cube))
        """
        Pick up the cube
        """
        

    async def do_action(self):
        # TODO
        pass
        
    def go_to_next_state(self):
        return DropOffCubeState(self.robot, self.cube)

class DropOffCubeState(State):
    """
        Drive the cube to the drop off location, pass in which cube (i.e. A, B, C, D)

    """
    def __init__(self, robot, cube):
        self.robot = robot
        self.cube = cube
        print ('Current state: {} Cube: {}'.format(str(self), cube))
        """
        Drop off cube to the correct location
        A -> Drone
        B -> Airplane
        C -> Inspection
        D -> Place
        """
        
    async def do_action(self):
        # TODO
        pass
        
    def go_to_next_state(self):
        if self.cube == 'D':
            return CompletedState(self.robot)
        else:
            # Go to the next cube
            return PickUpPlanState(self.robot, chr(ord(self.cube) + 1))

class CompletedState(State):
    """
        Drive the cube to the drop off location, pass in which cube (i.e. A, B, C, D)
    """
    def __init__(self, robot):
        self.robot = robot
        print ('Current state: {} '.format(str(self)))
        """
        Do something with cozmo?
        """
        return
        
    async def do_action(self):
        pass

    def go_to_next_state(self):
        return self