# simple_device.py

from my_states import LocalizeState

class StateMachine(object):
    """ 
    A simple state machine that mimics the functionality of a device from a 
    high level.
    """

    def __init__(self, robot):
        """ Initialize the components. """
        self.robot = robot
        # Start with a default state.
        self.state = LocalizeState(self.robot, 'A')

    # def on_event(self, event, robot):
    #     """
    #     This is the bread and butter of the state machine. Incoming events are
    #     delegated to the given states which then handle the event. The result is
    #     then assigned as the new state.
    #     """

    #     # The next state will be the result of the on_event function.
    #     self.state = self.state.on_event(event, robot)
    #     self.state = self.state.return_to_idle()