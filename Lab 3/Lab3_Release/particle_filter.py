# Chingyeu Leekung
# Mingyang Zhu

from grid import *
from particle import Particle
from utils import *
from setting import *
import setting
import numpy as np

RANDOM_SAMPLE_SIZE = 250
TRANS_SIGMA_CONSTANT = 2 * (MARKER_TRANS_SIGMA ** 2)
ROT_SIGMA_CONSTANT = 2 * (MARKER_ROT_SIGMA ** 2)

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    dh = odom[2]
    for particle in particles:
        x, y, h = particle.xyh
        dx, dy = rotate_point(odom[0], odom[1], h)
        new_x, new_y, new_h = add_odometry_noise((x + dx, y + dy, h + dh), ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        updated_particle = Particle(new_x, new_y, new_h)
        motion_particles.append(updated_particle)
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []

    particles_with_prob = []
    weight_of_particles = []
    prob_of_particles = []
    normalize_factor = 0.0

    # update weights 
    for p in particles:
        if not grid.is_free(p.x, p.y) or not grid.is_in(p.x, p.y):
            particles_with_prob.append(p)
            weight_of_particles.append(0.0)
        else:
            prob = get_probability(p, measured_marker_list, grid)
            particles_with_prob.append(p)
            weight_of_particles.append(prob)
            normalize_factor += prob

    if normalize_factor == 0.0:
        prob_of_particles = [1 / len(weight_of_particles) for weight in weight_of_particles]
    else:
        prob_of_particles = [weight / normalize_factor for weight in weight_of_particles]

    # resample
    measured_particles = np.random.choice(particles_with_prob, \
        size=len(particles_with_prob) - RANDOM_SAMPLE_SIZE, replace=True, p=prob_of_particles)
    measured_particles = measured_particles.tolist()

    random_particles = Particle.create_random(count=RANDOM_SAMPLE_SIZE, grid=grid)

    return measured_particles + random_particles

def get_probability(particle, measured_marker_list, grid):
    marker_list = particle.read_markers(grid)

    marker_pairs = [] # (robot_marker, particle_marker)

    for robot_marker in measured_marker_list: 
        smallest_dist = float('inf')
        if len(marker_list) > 0:
            closest_robot_marker = robot_marker
            closest_particle_marker = marker_list[0]
            for particle_marker in marker_list:
                dist = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
                if dist < smallest_dist:
                    closest_robot_marker = robot_marker
                    closest_particle_marker = particle_marker
                    smallest_dist = dist
            marker_pairs.append((closest_robot_marker, closest_particle_marker))
            marker_list.remove(closest_particle_marker)

    prob = 1.0
    for robot_marker, particle_marker in marker_pairs:
        distance = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
        angle = diff_heading_deg(robot_marker[2], particle_marker[2])
        expr1 = (distance ** 2) / TRANS_SIGMA_CONSTANT
        expr2 = (angle ** 2) / ROT_SIGMA_CONSTANT
        prob_match = math.exp(-1 * (expr1 + expr2))
        prob_no_match = setting.SPURIOUS_DETECTION_RATE * setting.DETECTION_FAILURE_RATE
        prob *= max((prob_match, prob_no_match))

    prob *= setting.SPURIOUS_DETECTION_RATE ** (len(measured_marker_list) - len(marker_pairs))

    prob *= setting.DETECTION_FAILURE_RATE ** len(marker_list)

    return prob