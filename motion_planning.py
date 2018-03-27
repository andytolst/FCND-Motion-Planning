import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import heuristic, a_star_graph, create_grid_and_edges, nearest_point, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt

import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 2

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        first_line = open('colliders.csv').readline()
        lat_lon = dict(val.strip().split(' ') for val in first_line.split(','))

        # set home position to (lon0, lat0, 0)
        self.set_home_position(float(lat_lon['lon0']), float(lat_lon['lat0']), 0)
        
        # retrieve current global position
        # convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        # Create Voronoi graph from obstacles
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid
        # convert start position to current position rather than map center
        grid_start = (-north_offset + int(local_position[0]), -east_offset + int(local_position[1]))
        
        # Set goal as some arbitrary position on the grid
        # adapt to set goal as latitude / longitude position and convert

        # some possible destinations on the map
        destinations = [ [-122.39324423, 37.79607305], 
                         [-122.39489652, 37.79124152], 
                         [-122.40059743, 37.79272177], 
                         [-122.39625334, 37.79478161] ]
        
        # randomly choose one of destinations
        destination = destinations[np.random.randint(0, len(destinations))]
        destination_local = global_to_local([destination[0], destination[1], TARGET_ALTITUDE], self.global_home)

        # adjust destination coordinates on the grid
        grid_goal = (-north_offset + int(destination_local[0]), -east_offset + int(destination_local[1]))

        print('Local Start and Goal: ', grid_start, grid_goal)
       
        # create Voronoi graph with the weight of the edges
        # set to the Euclidean distance between the points
        voronoi_graph = nx.Graph()

        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = np.linalg.norm(np.array(p2) - np.array(p1))
            voronoi_graph.add_edge(p1, p2, weight=dist )

        # find the nearest points on the graph for start and the goal
        voronoi_start = nearest_point(grid_start, voronoi_graph)
        voronoi_finish = nearest_point(grid_goal, voronoi_graph)

        # run A-star on the graph
        voronoi_path, voronoi_cost = a_star_graph(voronoi_graph, heuristic, voronoi_start, voronoi_finish)
        voronoi_path.append(grid_goal)

        # prune path - from Lesson 6.5
        print('Original path len: ', len(voronoi_path))
        voronoi_path = prune_path(voronoi_path)
        print('Pruned path len: ', len(voronoi_path))

        # Convert path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, TARGET_ALTITUDE, 0] for p in voronoi_path]

        # Set self.waypoints
        self.waypoints = waypoints
        
        # send waypoints to sim
        self.send_waypoints()

        ## Uncomment this to plot the graph and path #####################################
        #plt.rcParams['figure.figsize'] = 12, 12
        #plt.imshow(grid, origin='lower', cmap='Greys') 
        #
        ## graph
        #for e in edges:
        #    p1 = e[0]
        #    p2 = e[1]
        #    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
        #
        ## path
        #for e in voronoi_path:
        #    plt.plot(e[1], e[0], 'go')
        #
        ## start and goal 
        #plt.plot(grid_start[1], grid_start[0], 'bo')
        #plt.plot(grid_goal[1], grid_goal[0], 'ro')
        #
        #plt.xlabel('EAST')
        #plt.ylabel('NORTH')
        #plt.show()
        ###################################################################################


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
