import cozmo
import math
import sys
import time
import numpy as np

from cmap import *
from gui import *
from utils import *

from random import random as rd, choice
import numpy as np

MAX_NODES = 20000


from go_to_goal_cozmo import *


def step_from_to(node0, node1, limit=20):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    if get_dist(node0, node1) < limit:
    # 4. Note: remember always return a Node object
        return node1
    else:
        theta = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        # print("{} -> {}".format( node0.coord ,(node0.x + limit * math.cos(theta), node0.y + limit * math.sin(theta)) ))
        return Node((node0.x + limit * math.cos(theta), node0.y + limit * math.sin(theta)))
    ############################################################################

    
    


def node_generator(cmap):
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    

    rand_node = None
    while rand_node is None or not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        if rd() < 0.05:
            # print("generating a target node")
            target_coord = choice(cmap.get_goals()).coord
        else:
            # print('generate a random node')
            target_coord = (rd()*cmap.width, (rd()*cmap.height))
        rand_node = Node(target_coord)
    #temporary cod below to be replaced
    return rand_node
    ############################################################################
    


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
        
        #temporary code below to be replaced
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        min_dist = float('inf')
        for node in cmap.get_nodes():
            dist = get_dist(node, rand_node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        new_node = step_from_to(nearest_node, rand_node)


        if cmap.is_inbound(new_node) and not cmap.is_inside_obstacles(new_node):
            if not cmap.is_collision_with_obstacles((nearest_node, new_node)):
                cmap.add_node(new_node)
                cmap.add_path(nearest_node, new_node)
        ########################################################################
        
        
        # time.sleep(0.01)
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

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions. Potential pseudcode is below

    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    #Add final position as goal point to cmap, with final position being defined as a point that is at the center of the arena 
    #you can get map width and map weight from cmap.get_size()
    map_width, map_height = cmap.get_size()
    final_pos = (map_width / 2, map_height / 2)

    #reset the current stored paths in cmap
    cmap.reset_paths()
    cmap.add_goal(Node(final_pos))
    #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
    RRT(cmap, cmap.get_start())

    cozmo_pos = cmap.get_start()
    cozmo_angle = 0    
    #get path from the cmap
    # paths = cmap.get_path()
    paths = cmap.get_smooth_path()
    
    
    #marked and update_cmap are both outputted from detect_cube_and_update_cmap(robot, marked, cozmo_pos).

    #and marked is an input to the function, indicating which cubes are already marked
    #So initialize "marked" to be an empty dictionary and "update_cmap" = False
    marked = {}
    update_cmap = False
    update_cmap , _goal_center , marked = await detect_cube_and_update_cmap(robot, marked, cozmo_pos)
    
    last_subpath = cmap.get_start()
    #while the current cosmo position is not at the goal:
    while cozmo_pos != final_pos:
        #break if path is none or empty, indicating no path was found
        if paths is None or len(paths) <= 0:
            break
        
        

        # Get the next node from the path
        #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
        #you can calculate the angle to turn through a trigonometric function
        subpath = paths.pop(0) # of type (from_node, to_node)
        print("path: {}".format(paths))
        theta = np.arctan2(subpath.y - last_subpath.y, subpath.x - last_subpath.x)
        distance = get_dist(subpath, last_subpath)
        last_subpath = subpath
        await robot.turn_in_place(angle=cozmo.util.Angle(radians=theta), speed=cozmo.util.Angle(radians=1)).wait_for_completed()
        await robot.drive_straight(cozmo.util.distance_mm(distance_mm=distance), cozmo.util.speed_mmps(30)).wait_for_completed()
        await robot.turn_in_place(angle=cozmo.util.Angle(radians=-theta), speed=cozmo.util.Angle(radians=1)).wait_for_completed() 
            
        # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle 
        cozmo_pos = subpath.coord
    
        # Set new start position for replanning with RRT
        cmap.set_start(Node(coord=cozmo_pos))

        #detect any visible obstacle cubes and update cmap
        update_cmap , _goal_center , marked = await detect_cube_and_update_cmap(robot, marked, cozmo_pos)

        
        #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths 
        if update_cmap:
            cmap.reset_paths()
            RRT(cmap, cmap.get_start())
            paths = cmap.get_path()
    
    
    
    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabHappy).wait_for_completed()
    ########################################################################
    
    
    
    
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
    t = np.zeros((3,3))
    t[0, 0] = math.cos(local_angle)
    t[0, 1] = -math.sin(local_angle)
    t[0, 2] = local_origin.x
    t[1, 0] = math.sin(local_angle)
    t[1, 1] = math.cos(local_angle)
    t[1, 2] = local_origin.y
    t[2, 2] = 1

    new_coord = t @ (np.array(node.x, node.y, 1).T)

    #temporary code below to be replaced
    new_node = Node((new_coord.reshape(-1)[0], new_coord.reshape(-1)[1]))
    return new_node
    ########################################################################


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
    cube_padding = 40.
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

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


async def dock_with_cube(robot: cozmo.robot.Robot):

    print("Cozmo is waiting until he sees a cube.")
    cube = await robot.world.wait_for_observed_light_cube()


    # print("Cozmo found a cube, and will now attempt to dock with it:")
    # # Cozmo will approach the cube he has seen
    # # using a 180 approach angle will cause him to drive past the cube and approach from the opposite side
    # # num_retries allows us to specify how many times Cozmo will retry the action in the event of it failing
    await robot.dock_with_cube(cube, approach_angle=cozmo.util.degrees(0), num_retries=2).wait_for_completed()

    await robot.set_lift_height(1).wait_for_completed()

    await robot.turn_in_place(cozmo.util.degrees(45),
                                speed=cozmo.util.Angle(degrees=60)).wait_for_completed()    

async def init(robot: cozmo.robot.Robot):
    
    await robot.set_lift_height(0).wait_for_completed()



class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(init)
        # cozmo.run_program(localization, use_viewer=False)

        cozmo.run_program(dock_with_cube)

        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class DeliveryThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset_paths()
        stopevent.set()




if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/path_planning_map.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/path_planning_map.json", node_generator)
        sim = DeliveryThread()
        sim.start()
    localization_gui.show_particles(pf.particles)
    localization_gui.show_mean(0, 0, 0)
    localization_gui.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()