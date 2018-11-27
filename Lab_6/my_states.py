import sys
sys.path.append('../')
from state import State
import time
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
import cozmo.behavior
# import classifyimage
import time

def most_common(lst):
    return max(set(lst), key=lst.count)

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

        self.go_to_next_state()

    def go_to_next_state(self):
        return PickUpPlanState(self.robot, self.cube)

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