import argparse
import networkx as nx
import time
import numpy as np
from motion_planning import States, MotionPlanning
from planning_utils import closest_point, create_grid_and_edges, a_star_graph, prune_path
import numpy.linalg as LA
from udacidrone.connection import MavlinkConnection
from udacidrone.frame_utils import global_to_local

class GraphMotionPlanning(MotionPlanning):
    def __init__(self, connection, goal_lon, goal_lat, goal_alt = 0):
        super(GraphMotionPlanning, self).__init__(connection, goal_lon, goal_lat)
        self.goal_alt = float(goal_alt) if goal_alt else 0.0


    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        firstline = open('colliders.csv').readline().split(',')
        lat0 = float(firstline[0].strip("lat0 "))
        lon0 = float(firstline[1].strip("lon0 "))
        print ('lat0 = {}, lon0 = {}'.format(lat0, lon0))

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        local_position = global_to_local(self.global_position, self.global_home)
 
        # TODO: convert to current local position using global_to_local()
         
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
 
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        offset = np.array([north_offset, east_offset, 0])

        #create the graph with the weight of the edges using euclidean distance between the points
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        start_pos = closest_point(G, local_position - offset)

        local_goal = global_to_local([float(self.goal_lon), float(self.goal_lat), float(self.goal_alt)], self.global_home) - offset
        goal_pos = closest_point(G, local_goal)

        print('Local Start and Goal: ', start_pos, goal_pos)
        path, _ = a_star_graph(G, start_pos, goal_pos)
        path = prune_path(path, 0.5)

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=str, default='', help='Goal longitude')
    parser.add_argument('--goal_lat', type=str, default='', help='Goal latitude')
    parser.add_argument('--goal_alt', type=str, default='', help='Goal Altitude')

    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = GraphMotionPlanning(conn, args.goal_lon, args.goal_lat, args.goal_lat)
    time.sleep(1)

    drone.start()