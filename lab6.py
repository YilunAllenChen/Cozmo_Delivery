###############
# Name1: Kaijun Lin
# Name2: Junyan Mao
###############
try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass

import cozmo
import math
import sys
from cozmo.util import degrees, distance_mm, speed_mmps

import time
import threading
import numpy as np
from sklearn import svm, metrics
from skimage import color, io, feature, filters, exposure
from joblib import dump, load
from PIL import ImageDraw, ImageFont

from markers import detect, annotator
from localize_grid import CozGrid
from localize_gui import GUIWindow
from localize_setting import *
from localize_utils import *
from particle import Particle, Robot
from cmap import *
from rrt_gui import *
from rrt_utils import *

stop = False
clf = load('SVCMOD.joblib')

Map_filename = 'localize_map.json'
grid = CozGrid(Map_filename)
local_gui = GUIWindow(grid, show_camera=True)
particles = Particle.create_random(1, grid)

HANDS_POS = (0, 234.1, 0)
PLACE_POS = (650, 310.3, 180)
robot_pos = [0, 0, 0]

# in grid unit
PICKUP_CORNER = (107.95, 150, 0)
PICKUP_CENTER = (107.95, 342.05, 0)
DELIVER_CENTER = (542.05, 342.05, 0)
ARENA_CENTER = (325, 225, 0)

MAX_NODES = 20000

# RRT Code
def step_from_to(node0, node1, limit=75):
    if get_dist(node0, node1) < limit:
        return node1
    else:
        angle = math.atan2(node0.y - node1.y, node0.x - node1.x)
        newX = node0.x + limit * math.cos(angle)
        newY = node0.y + limit * math.sin(angle)
        newNode = Node((newX, newY))
        return newNode
    
def node_generator(cmap):
    rand_node = None
    x = np.random.uniform(0, cmap.width)
    y = np.random.uniform(0, cmap.height)
    rand_node = Node((x, y))
    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        x = np.random.uniform(0, cmap.width)
        y = np.random.uniform(0, cmap.height)
        rand_node = Node((x, y))
    randProb = np.random.randint(0, 100)
    if randProb >= 95:
        return Node((cmap.get_goals()[0][0], cmap.get_goals()[0][1]))
    return rand_node

def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        nearest_dist = float('inf')
        for node in cmap.get_nodes():
            if get_dist(node, rand_node) < nearest_dist:
                nearest_node = node
                nearest_dist = get_dist(node, rand_node)
        newNode = step_from_to(nearest_node, rand_node, 15)
        cmap.add_node(newNode)
        cmap.add_path(nearest_node, newNode)
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
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

def get_global_node(local_angle, local_origin, node):
    new_node = None
    transform_matrix = np.ndarray(shape=(3, 3), dtype=float)
    transform_matrix[0][0] = math.cos(local_angle)
    transform_matrix[0][1] = -math.sin(local_angle)
    transform_matrix[0][2] = local_origin.x

    transform_matrix[1][0] = math.sin(local_angle)
    transform_matrix[1][1] = math.cos(local_angle)
    transform_matrix[1][2] = local_origin.y

    transform_matrix[2][0] = 0
    transform_matrix[2][1] = 0
    transform_matrix[2][2] = 1
    node_coord = np.ndarray(shape=(3, 1), dtype=float)
    node_coord[0][0] = node.x 
    node_coord[1][0] = node.y
    node_coord[2][0] = 1
    origin_coord = np.dot(transform_matrix, node_coord)
    new_node = Node((origin_coord[0][0], origin_coord[1][0]))
    return new_node

def classify(image):
    image = filters.gaussian(image, sigma=0.4)
    f = feature.hog(image, orientations=10, pixels_per_cell=(48, 48),
                    cells_per_block=(4, 4), feature_vector=True, block_norm='L2-Hys')
    to_predict = []
    to_predict.append(f)
    result = clf.predict(to_predict)
    return result

def run(robot: cozmo.robot.Robot):
    global stop
    global cmap, stopevent

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    marker_annotator = annotator.MarkerAnnotator(robot.world.image_annotator)
    robot.world.image_annotator.add_annotator('Marker', marker_annotator)

    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float)

    angle_diff = 0
    initial_heading = 0

    # LOCALIZATION
    while not stop:
        if not robot.world.latest_image:
            continue
        image = np.array(robot.world.latest_image.raw_image)
        image = color.rgb2gray(image)
        markers = detect.detect_markers(image, camera_settings)
        marker_annotator.markers = markers
        result = classify(image)
        robot.say_text(result[0]).wait_for_completed()
        if result[0] == 'hands' or result[0] == 'place':
            robot.stop_all_motors()
            x, y, h = markers[0]['xyh']
            if h > 210 or h < 150:
                robot.turn_in_place(degrees(-10), speed=degrees(60)).wait_for_completed()
                continue
            if result[0] == 'hands':
                robot_pos = [HANDS_POS[0] + x, HANDS_POS[1] + y, HANDS_POS[2] + h]
                angle_diff = robot.pose.rotation.angle_z.degrees + robot_pos[2]
                robot.turn_in_place(degrees(-50), speed=degrees(60)).wait_for_completed()
            elif result[0] == 'place':
                robot_pos = [PLACE_POS[0] - x, PLACE_POS[1] + y, PLACE_POS[2] - h]
                angle_diff = robot.pose.rotation.angle_z.degrees + robot_pos[2]
                robot.turn_in_place(degrees(140), speed=degrees(60)).wait_for_completed()
            # robot_pos = [robot_pos[0] / 25, robot_pos[1] / 25, robot_pos[2]]
            break
        robot.turn_in_place(degrees(20), speed=degrees(40)).wait_for_completed()
        time.sleep(0.1)
    # Localization GUI
    # local_gui.show_robot(Robot(robot_pos[0], robot_pos[1], robot_pos[2]))
    # local_gui.updated.set()
    
    robot.say_text('Ready for pickup.').wait_for_completed()
    time.sleep(0.5)
    # Turn off camera stream
    robot.camera.image_stream_enabled = False
    found_cube = False
    lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)

    # Picking up and placing the first cube
    while not found_cube:
        cube = None
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=30)
        except:
            print("cube not found")
        finally:
            lookaround.stop()
        if cube is None:
            continue
        found_cube = True
        robot.pickup_object(cube, num_retries=3).wait_for_completed()
        robot.drive_straight(distance_mm(50), speed_mmps(50)).wait_for_completed()
        # robot.place_object_on_ground_here(cube, num_retries=3).wait_for_completed()
        robot.turn_in_place(degrees(-robot.pose.rotation.angle_z.degrees), speed=degrees(60)).wait_for_completed()
        robot.turn_in_place(degrees(angle_diff - robot_pos[2]), speed=degrees(60)).wait_for_completed()
        x = robot.pose.position.x
        y = robot.pose.position.y

        robot_pos[0] = ARENA_CENTER[0] + x
        robot_pos[1] = ARENA_CENTER[1] + y
        robot_pos[2] = 0
        # local_gui.show_robot(Robot(robot_pos[0], robot_pos[1], robot_pos[2]))
        # local_gui.updated.set()
        curr_pos = robot_pos
        cmap.reset()
        # cmap.set_start(Node(robot_pos))
        # RRT(cmap, Node((robot_pos[0], robot_pos[1])))
        cmap.set_start(Node(PICKUP_CENTER))
        RRT(cmap, Node((PICKUP_CENTER[0], PICKUP_CENTER[1])))
        final_path = cmap.get_smooth_path()
        index = 0
        while not stopevent.is_set() and index < len(final_path):
            node = final_path[index]
            end_pos = (node.x, node.y, 0)
            dist = ((end_pos[0] - curr_pos[0]) ** 2 + (end_pos[1] - curr_pos[1]) ** 2) ** 0.5
            angle = math.atan2(end_pos[1] - curr_pos[1], end_pos[0] - curr_pos[0])
            robot.turn_in_place(angle=cozmo.util.Angle(radians=angle), speed=degrees(60)).wait_for_completed()
            robot.drive_straight(distance_mm(dist), speed_mmps(50)).wait_for_completed()
            robot.turn_in_place(angle=cozmo.util.Angle(radians=-1*angle), speed=degrees(60)).wait_for_completed()
            index += 1
            curr_pos = end_pos
        robot.place_object_on_ground_here(cube, num_retries=3).wait_for_completed()

    for i in range(4):
        x = robot.pose.position.x
        y = robot.pose.position.y
        robot_pos[0] = ARENA_CENTER[0] + x
        robot_pos[1] = ARENA_CENTER[1] + y
        robot_pos[2] = 0
        cmap.reset()
        # cmap.set_start(Node(robot_pos))
        cmap.set_start(Node(DELIVER_CENTER))
        cmap.clear_goals()
        cmap.add_goal(Node(PICKUP_CORNER))
        # RRT(cmap, Node((robot_pos[0], robot_pos[1])))
        RRT(cmap, Node((DELIVER_CENTER[0], DELIVER_CENTER[1])))
        final_path = cmap.get_smooth_path()
        index = 0
        while not stopevent.is_set() and index < len(final_path):
            node = final_path[index]
            end_pos = (node.x, node.y, 0)
            dist = ((end_pos[0] - curr_pos[0]) ** 2 + (end_pos[1] - curr_pos[1]) ** 2) ** 0.5
            angle = math.atan2(end_pos[1] - curr_pos[1], end_pos[0] - curr_pos[0])
            robot.turn_in_place(angle=cozmo.util.Angle(radians=angle), speed=degrees(60)).wait_for_completed()
            robot.drive_straight(distance_mm(dist), speed_mmps(50)).wait_for_completed()
            robot.turn_in_place(angle=cozmo.util.Angle(radians=-1*angle), speed=degrees(60)).wait_for_completed()
            index += 1
            curr_pos = end_pos
        robot.say_text('Ready for pickup.').wait_for_completed()
        robot.turn_in_place(degrees(60), speed=degrees(60)).wait_for_completed()


        found_cube = False
        lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        while not found_cube:
            cube = None
            try:
                cube = robot.world.wait_for_observed_light_cube(timeout=30)
            except:
                print("cube not found")
            finally:
                lookaround.stop()
            if cube is None:
                continue
            found_cube = True
            robot.pickup_object(cube, num_retries=3).wait_for_completed()
            robot.drive_straight(distance_mm(50), speed_mmps(50)).wait_for_completed()
            # robot.place_object_on_ground_here(cube, num_retries=3).wait_for_completed()
            robot.turn_in_place(degrees(-robot.pose.rotation.angle_z.degrees), speed=degrees(60)).wait_for_completed()
            robot.turn_in_place(degrees(angle_diff - robot_pos[2]), speed=degrees(60)).wait_for_completed()
            x = robot.pose.position.x
            y = robot.pose.position.y
            robot_pos[0] = ARENA_CENTER[0] + x
            robot_pos[1] = ARENA_CENTER[1] + y
            robot_pos[2] = 0
            # local_gui.show_robot(Robot(robot_pos[0], robot_pos[1], robot_pos[2]))
            # local_gui.updated.set()
            curr_pos = robot_pos
            cmap.reset()
            cmap.clear_goals()
            cmap.add_goal(Node(DELIVER_CENTER))
            # cmap.set_start(Node(robot_pos))
            # RRT(cmap, Node((robot_pos[0], robot_pos[1])))
            cmap.set_start(Node(PICKUP_CENTER))
            RRT(cmap, Node((PICKUP_CENTER[0], PICKUP_CENTER[1])))
            final_path = cmap.get_smooth_path()
            index = 0
        while not stopevent.is_set() and index < len(final_path):
            node = final_path[index]
            end_pos = (node.x, node.y, 0)
            dist = ((end_pos[0] - curr_pos[0]) ** 2 + (end_pos[1] - curr_pos[1]) ** 2) ** 0.5
            angle = math.atan2(end_pos[1] - curr_pos[1], end_pos[0] - curr_pos[0])
            robot.turn_in_place(angle=cozmo.util.Angle(radians=angle), speed=degrees(60)).wait_for_completed()
            robot.drive_straight(distance_mm(dist), speed_mmps(50)).wait_for_completed()
            robot.turn_in_place(angle=cozmo.util.Angle(radians=-1*angle), speed=degrees(60)).wait_for_completed()
            index += 1
            curr_pos = end_pos
        robot.place_object_on_ground_here(cube, num_retries=3).wait_for_completed()
        
            
            



class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)
    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False
        cozmo.run_program(run, use_viewer=False)
        stopevent.set()

class RRTThread(threading.Thread):
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

    cmap = CozMap('rrt_map.json', node_generator)
    # sim = RRTThread()
    # sim.start()
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # local_gui.show_particles(particles)
    # local_gui.show_mean(0, 0, 0)
    # local_gui.start()
    
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()