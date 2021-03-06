''' Lab 6
Chingyeu Andy Leekung
Mingyang Zhu
'''

try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass
    
import sys

import cozmo
import datetime
import time
import numpy as np
from sklearn.externals import joblib
from skimage import io, feature, filters, exposure, color

from state_machine import StateMachine

async def run(sdk_conn):

    robot = await sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()

    # instantiate and run State Machine
    machine = StateMachine(robot)
    await machine.run()

if __name__ == '__main__':
    # device = StateMachine(None)

    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
