'''
This code implements the RRT* (Rapidly-exploring Random Tree Star) algorithm, 
an advanced version of the RRT algorithm used for path planning in robotics
and autonomous systems. Here's a summary:

1. RRTStar Class:

Inherits from a base RRT class.Contains a nested Node class with an additional
cost attribute for each node. The constructor initializes various 
parameters like start and goal positions, obstacle list, and 
algorithm-specific parameters (e.g., expand_dis, path_resolution,
goal_sample_rate, max_iter, connect_circle_dist, search_until_max_iter, robot_radius).

Implements several methods for RRT* functionality:
planning: Main method to find a path from start to goal. 
It iteratively builds a tree by randomly sampling points
and connecting them in a way that seeks the goal while avoiding obstacles.
choose_parent: Chooses the best parent for a new node considering the lowest cost path.
search_best_goal_node: Searches for the closest node to the goal.
find_near_nodes: Finds nearby nodes within a certain radius from a given node.
rewire: Optimizes the tree by changing parent nodes of some nodes to the new node if it provides a shorter path.
calc_new_cost and propagate_cost_to_leaves: Helper functions for cost calculations and updating the tree.

2. Collision Checking:

A static method check_collision checks if a path of a node collides with any obstacles, considering the robot's radius.

3. Main Function:

Defines an obstacle_list with positions and sizes of obstacles.
Initializes an RRTStar object with start and goal positions, the obstacle list, and other parameters.
Calls the planning method to find a path.
If a path is found, it is displayed using matplotlib.

The algorithm's key features include finding the most cost-effective path (considering the total length),
dynamically choosing parents for new nodes, and rewiring to ensure the path's optimality. The use of random
sampling for nodes, along with the goal-directed behavior influenced by goal_sample_rate, makes it effective
in complex environments. The visualization part (if show_animation is True) illustrates the algorithm's progress and the final path.
'''


import math
import sys
import matplotlib.pyplot as plt
import pathlib

from rrt import RRT

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False,
                 robot_radius=0.0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    """
    planning: Main method to find a path from start to goal. 
    It iteratively builds a tree by randomly sampling points
    and connecting them in a way that seeks the goal while avoiding obstacles.
    choose_parent: Chooses the best parent for a new node considering the lowest cost path.
    search_best_goal_node: Searches for the closest node to the goal.
    find_near_nodes: Finds nearby nodes within a certain radius from a given node.
    rewire: Optimizes the tree by changing parent nodes of some nodes to the new node if it provides a shorter path.
    calc_new_cost and propagate_cost_to_leaves: Helper functions for cost calculations and updating the tree.
    """
    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            # Progress printout
            print("Iter:", i, ", number of nodes:", len(self.node_list)) 

            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(node_list=self.node_list, rnd_node=rnd)
            new_node = self.steer(from_node=self.node_list[nearest_ind], to_node=rnd, extend_length=self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = self.calc_new_cost(from_node=near_node, to_node=new_node)

            # If there is no collision, branch to the shortest path (if applicable) and rewire
            if self.check_collision(node=new_node, obstacleList=self.obstacle_list, robot_radius=self.robot_radius):
                near_inds = self.find_near_nodes(new_node=new_node)
                new_node = self.choose_parent(new_node=new_node, near_inds=near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)


            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    path = self.generate_final_course(last_index)
                    smooth_path = self.smooth_trajectory(path=path, window_size=5)
                    return smooth_path

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            path = self.generate_final_course(last_index)
            smooth_path = self.smooth_trajectory(path=path, window_size=5)
            return smooth_path

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # Search nearest cost in near_inds
        costs = []
        for i in near_inds:
            if not self.check_collision(node=new_node, obstacleList=self.obstacle_list, robot_radius=self.robot_radius):
                costs.append(float("inf"))
            else:
                costs.append(self.calc_new_cost(self.node_list[i], new_node))

        # Find the minimum cost
        min_cost = min(costs)

        # If there is no new parent
        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None


        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(from_node=self.node_list[min_ind], to_node=new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        """
        Searches for the closest node to the goal
        """

        # Calculate distances to the goal pose
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]

        # Look for the node(s) that has distance smaller than expand_dis, there might be more than one
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        # Check if there is collision by connecting this node(s) with the goal pose
        safe_goal_inds = []
        for goal_ind in goal_inds:
            
            t_node = self.steer(from_node=self.node_list[goal_ind], to_node=self.goal_node)
            if self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        # If none are collision free, keep searching
        if not safe_goal_inds:
            return None

        safe_goal_costs = [self.calc_new_cost(self.node_list[i], self.goal_node) for i in safe_goal_inds]

        # Take the one with minimum cost
        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # If expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(from_node=new_node, to_node=near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(from_node=near_node, to_node=new_node)

            no_collision = self.check_collision(node=edge_node, obstacleList=self.obstacle_list, robot_radius=self.robot_radius)
            improved_cost = edge_node.cost < near_node.cost

            # If not collision and lower cost, then perform re-wiring
            if no_collision and improved_cost:
                for node in self.node_list:
                    if node.parent == self.node_list[i]:
                        node.parent = edge_node
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def main():
    print("Start " + __file__)

    # Define the list of virtual obstacles, see below for the entries
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]
    # obstacle_list = [
    #     (2, 2, 1),
    #     (6, 2, 2),
    #     (4, 4, 2),
    #     (8, 3, 1),
    #     (10, 4, 1),
    #     (5, 7, 2),
    #     (10, 10, 2),
    #     (6, 12, 1),
    #     (12, 7, 2)
    #     ]

    # Set Initial parameters, refer to the explanation at the top of this file
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.2,
        max_iter=1000,
        connect_circle_dist=40,
        goal_sample_rate=30,
        path_resolution=1)
    
    # Search the path using RRT*
    path = rrt_star.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--', linewidth = '2.0')
            plt.grid(True)
            plt.show()


if __name__ == '__main__':
    main()
