## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

- planning_utils.py, provide utility function which motion_planning.py needs to perform a star search, and the code to read in and build the grid from csv file
- motion_planning.py provides the basic functionality for the state transition logic for the controller, as well as skeleton code for the planning step which invoke a search to calcuate waypoints, and send the waypoint to the controller

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

<pre>
        firstline = open('colliders.csv').readline().split(',')
        lat0 = float(firstline[0].strip("lat0 "))
        lon0 = float(firstline[1].strip("lon0 "))
        print ('lat0 = {}, lon0 = {}'.format(lat0, lon0))
</pre>        	

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

<pre>
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        local_position = global_to_local(self.global_position, self.global_home)
</pre>

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

first read in the ostacle map, and build a grid for a particular altitude and safety margin around obstacles
<pre>
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset, _ = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
</pre>

then translate the position to the grid coordinate
<pre>
        # Define starting point on the grid (this is just grid center)
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
</pre>

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

the longtitude and latitude of the goal position can be pass in via the command line as follows for the 2D version

python motion_planning.py --goal_lon -122.39725 --goal_lat 37.79392

if the goal location is not passed in, then it will pick a random location as the goal position using the follow method

<pre>
        angular_range = 0.002 
        if not self.goal_lon or not self.goal_lat:
            print ('pick random goal location')
            lat_goal = lat0 + (2*np.random.rand()-1)*angular_range
            lon_goal = lon0 + (2*np.random.rand()-1)*angular_range
        else:
            lon_goal = float(self.goal_lon)
            lat_goal = float(self.goal_lat)
            print ('use pass arg', lon_goal, lat_goal)
        while True:
            global_goal_pose = np.array([lon_goal, lat_goal, 0])
            local_goal = global_to_local(global_goal_pose, self.global_home)
            goal_x, goal_y = int(local_goal[0]-north_offset), int(local_goal[1]-east_offset)
            if grid[goal_x, goal_y] == 0:
               break
            #pick random location if previous pick is an obstacle
            lat_goal = lat0 + (2*np.random.rand()-1)*angular_range
            lon_goal = lon0 + (2*np.random.rand()-1)*angular_range

        grid_goal = (goal_x, goal_y)
</pre>        


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2).

first add the diagonally moving actions to the list of possible actions
<pre>
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
</pre>

and then add the logic to detect when those diagnoally moving actions is impossible at a given grid location
<pre>
    if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y + 1 > m or grid[x+1, y+1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y -1 < 0 or grid[x-1, y-1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if x + 1 > n or y - 1 < 0 or grid[x+1, y-1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)

    return valid_actions
</pre>

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

the code calculates the determinant of three consecutive points to determine if they lie on the same line, and the middle point can be pruned as . anything with determinant less than epsilon (defined to be 0.5 here), is being removed.

<pre>
#path pruning
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path, eps=1e-6):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3, eps):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
</pre>

## Build Voronoi graph to construct graph representing the path

create voroni graph based on the obstacles, and then build a graph, and use a star and prune procedures to generate the path.
use the following command to run

python graph_motion_planning.py --goal_lon -122.40195876 --goal_lat 37.79673913 --goal_alt 0

<pre>
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    grid, north_min, east_min, points = create_grid(data, drone_altitude, safety_distance)
    #create a voroni graph based on location of obstacle centres
    graph = Voronoi(points)
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges, north_min, east_min
</pre>    	

 

