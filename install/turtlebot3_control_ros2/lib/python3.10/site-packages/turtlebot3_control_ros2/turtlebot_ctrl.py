#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")
        self.laser = LaserScan()
        self.odom = Odometry()
        self.map = np.array([	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
        self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
        self.timer = self.create_timer(0.1, self.cmd_vel_pub)
        
        self.target_position = None
        self.map_resolution = 4
        self.robot_radius = 0.2  # metros
        self.obstacle_threshold = 0.5  # metros
        self.state = "FIND_TARGET"
        self.stuck_counter = 0
        self.stuck_threshold = 50  # número de iterações antes de considerar preso
        
    def find_next_target(self):
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i][j] == 1:
                    return i, j
        return None
    
    def is_obstacle_ahead(self):
        if len(self.laser.ranges) == 0:
            return True
        front_angle = len(self.laser.ranges) // 2
        return min(self.laser.ranges[front_angle-10:front_angle+10]) < self.obstacle_threshold
    
    def find_clear_direction(self):
        if len(self.laser.ranges) == 0:
            return 0
        max_range = max(self.laser.ranges)
        return self.laser.ranges.index(max_range) * self.laser.angle_increment
    
    def cmd_vel_pub(self):
        current_x = int(-self.odom.pose.pose.position.x * self.map_resolution + self.map.shape[0]/2)
        current_y = int(-self.odom.pose.pose.position.y * self.map_resolution + self.map.shape[1]/2)
        
        # Atualizar a posição atual como visitada
        if 0 <= current_x < self.map.shape[0] and 0 <= current_y < self.map.shape[1]:
            self.map[current_x][current_y] = 2
        
        msg = Twist()
        
        if self.state == "FIND_TARGET":
            self.target_position = self.find_next_target()
            if self.target_position is None:
                self.get_logger().info("Mapa completamente explorado!")
                return
            self.state = "MOVE_TO_TARGET"
        
        elif self.state == "MOVE_TO_TARGET":
            if self.is_obstacle_ahead():
                self.state = "AVOID_OBSTACLE"
            else:
                dx = self.target_position[0] - current_x
                dy = self.target_position[1] - current_y
                target_angle = math.atan2(dy, dx)
                _, _, current_angle = self.euler_from_quaternion(self.odom.pose.pose.orientation)
                
                angle_diff = target_angle - current_angle
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                if abs(angle_diff) > 0.1:
                    msg.angular.z = 0.5 if angle_diff > 0 else -0.5
                else:
                    msg.linear.x = 0.2
                
                # Verificar se chegou ao alvo
                if abs(dx) <= 1 and abs(dy) <= 1:
                    self.state = "FIND_TARGET"
        
        elif self.state == "AVOID_OBSTACLE":
            clear_direction = self.find_clear_direction()
            msg.angular.z = 0.5 if clear_direction > 0 else -0.5
            if not self.is_obstacle_ahead():
                self.state = "MOVE_TO_TARGET"
        
        self.publish_cmd_vel.publish(msg)
        
        # Verificar se está preso
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        if self.stuck_counter > self.stuck_threshold:
            self.get_logger().warn("Robô parece estar preso. Tentando recuperar...")
            self.state = "AVOID_OBSTACLE"
            self.stuck_counter = 0
        
        # Logar informações
        coverage = 100 * float(np.count_nonzero(self.map == 2)) / (np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))
        self.get_logger().info(f"Posição: [{current_x}, {current_y}]. Cobertura: {coverage:.2f}%")
    
    def callback_laser(self, msg):
        self.laser = msg
    
    def callback_odom(self, msg):
        self.odom = msg
    
    @staticmethod
    def euler_from_quaternion(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()