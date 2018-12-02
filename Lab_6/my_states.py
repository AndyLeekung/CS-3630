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


        self.go_to_next_state()

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
        

        self.go_to_next_state()
        
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
        
        self.go_to_next_state()
        
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
        
        
    def go_to_next_state(self):
        return self