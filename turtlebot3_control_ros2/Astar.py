#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import heapq
import random

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")

        self.laser = LaserScan()
        self.odom = Odometry()

        self.map = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])

        self.map_resolution = 4
        self.current_pos = None
        self.goal = None
        self.path = []

        self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
        self.timer = self.create_timer(0.5, self.explore_and_move)

    def callback_laser(self, msg):
        self.laser = msg

    def callback_odom(self, msg):
        self.odom = msg
        self.update_current_position()

    def update_current_position(self):
        index_x = -int(self.odom.pose.pose.position.x * self.map_resolution)
        index_y = -int(self.odom.pose.pose.position.y * self.map_resolution)

        index_x += int(self.map.shape[0] / 2)
        index_y += int(self.map.shape[0] / 2)

        index_x = max(1, min(index_x, self.map.shape[0] - 1))
        index_y = max(1, min(index_y, self.map.shape[0] - 1))
        
        if (index_x < 1): index_x = 1
        if (index_x > self.map.shape[0]-1): index_x = self.map.shape[0]-1
        if (index_y < 1): index_y = 1
        if (index_y > self.map.shape[0]-1): index_y = self.map.shape[0]-1

        # Marca a célula visitada e imprime o progresso
        if (self.map[index_x][index_y] == 1):
            self.map[index_x][index_y] = 2
            self.get_logger().info("Outra parte alcançada... percentual coberto: " + 
                                   str(100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) 
                                                                                + np.count_nonzero(self.map == 2))))
            self.get_logger().info("Mapa discreto atualizado")
            self.get_logger().info("\n"+str(self.map))

        self.current_pos = (index_x, index_y)

    def heuristic(self, a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    def get_neighbors(self, node):
        self.update_current_position()
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dir in dirs:
            neighbor = (node[0] + dir[0], node[1] + dir[1])
            if 0 <= neighbor[0] < self.map.shape[0] and 0 <= neighbor[1] < self.map.shape[1]:
                if self.map[neighbor[0]][neighbor[1]] != 0:
                    result.append(neighbor)
        return result

    def astar(self, start, goal):
        self.update_current_position()
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def find_nearest_unexplored(self):
        self.update_current_position()
        unexplored = np.argwhere(self.map == 1)
        if len(unexplored) == 0:
            return None
        distances = [self.heuristic(self.current_pos, point) for point in unexplored]
        return tuple(unexplored[np.argmin(distances)])

    def explore_and_move(self):
        self.update_current_position()

        if self.current_pos is None or not self.laser.ranges:
            return

        msg = Twist()

        obstacle_right = self.laser.ranges[:30]
        obstacle_left = self.laser.ranges[-30:]

        if min(obstacle_right) < random.uniform(0.3, 0.5):
            msg.linear.x = 0.0
            msg.angular.z = -0.4
        elif min(obstacle_left) < random.uniform(0.3, 0.5):
            msg.linear.x = 0.0
            msg.angular.z = 0.4
        else:
            if not self.path:
                self.goal = self.find_nearest_unexplored()
                if self.goal is None:
                    self.get_logger().info("Exploration complete!")
                    return
                self.path = self.astar(self.current_pos, self.goal)

            if self.path:
                next_pos = self.path[0]
                dx = next_pos[0] - self.current_pos[0]
                dy = next_pos[1] - self.current_pos[1]
                distance = math.sqrt(dx**2 + dy**2)

                if distance > 0.1:
                    self.update_current_position()
                    msg.linear.x = min(0.3 * distance, 0.3)
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.path.pop(0)

        self.publish_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()