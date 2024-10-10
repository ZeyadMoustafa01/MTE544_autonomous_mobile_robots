from mapUtilities import *
from rrt_star import RRTStar
import sys
from a_star import *
import time

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3

# TODO Modify this class so that is uses the RRT* planner with virtual obstacles

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose=None, endPose=None):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        self.m_utilites=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilites.make_likelihood_field()
        
        obstacle_list_1 = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
        ]
        
        obstacle_list_2 = [
        (2, 2, 1),
        (6, 2, 2),
        (4, 4, 2),
        (8, 3, 1),
        (10, 4, 1),
        (5, 7, 2),
        (10, 10, 2),
        (6, 12, 1),
        (12, 7, 2)
        ]
        
        self.rrt_star = RRTStar(
            start=[0, 0],
            goal=[14, 10],
            rand_area=[-2, 15],
            obstacle_list=obstacle_list_2,
            expand_dis=1,
            robot_radius=0.2,
            max_iter=1000,
            connect_circle_dist=40.0,
            goal_sample_rate=30,
            path_resolution=1
        )

        #TODO Remember to initialize the rrt_star
        
    
    def trajectory_planner(self, startPoseCart=None, endPoseCart=None, type=None):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #### If not using the map (no bonus), you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)
        scale_factor = 1 # this is the downsample scale, if set 2, it will downsample the map by half, and if set x, it will do the same as 1/x

        if startPoseCart and endPoseCart:
            startPose=self.m_utilites.position_2_cell(startPoseCart)
            endPose=self.m_utilites.position_2_cell(endPoseCart)
            
            startPose = [int(i/scale_factor) for i in startPose]
            endPose   = [int(j/scale_factor) for j in endPose]
            mazeOrigin = self.m_utilites.position_2_cell([0,0])
        
        start_time = time.time()

        # TODO This is for A*, modify this part to use RRT*
        if type == A_STAR_PLANNER:
            path = search(self.costMap, startPose, endPose, scale_factor)
        elif type == RRT_STAR_PLANNER:
            path = self.rrt_star.planning(animation=False)
        
        if path is None:
            print("Cannot find path")
            sys.exit(1)


        end_time = time.time()

        # This will display how much time the search algorithm needed to find a path
        print(f"the time took for a_star calculation was {end_time - start_time}")

        # path_ = [[x*scale_factor, y*scale_factor] for x,y in path]
        # Path = np.array(list(map(self.m_utilites.cell_2_position, path_)))

        # TODO Smooth the path before returning it to the decision maker
        # this can be in form of a function that you can put in the utilities.py 
        # or add it as a method to the original rrt.py 
        path = path[::-1]
        np.save(file="data/obstacle2_goal2.npy", arr=path)
        print(f"path is {path}")
        return path


if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can do your test here ...

