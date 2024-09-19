# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# import numpy as np

# class TurtlebotCtrl(Node):
#     def __init__(self):
#         super().__init__("TurtlebotCtrl")

#         self.laser = LaserScan()
#         self.odom = Odometry()
#         self.obstacle_detected = False
#         self.obstacle_avoidance_mode = False
#         self.obstacle_avoidance_counter = 0
#         self.escape_mode = False
#         self.escape_counter = 0

#         self.map = np.array([   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#                             ])

#         self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
#         self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
#         self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
#         self.timer = self.create_timer(0.5, self.cmd_vel_pub)

#     def cmd_vel_pub(self):
#         map_resolution = 4

#         # Mantendo a lógica original para a conversão de coordenadas
#         index_x = -int(self.odom.pose.pose.position.x * map_resolution)
#         index_y = -int(self.odom.pose.pose.position.y * map_resolution)

#         index_x += int(self.map.shape[0] / 2)
#         index_y += int(self.map.shape[0] / 2)

#         if index_x < 1: index_x = 1
#         if index_x > self.map.shape[0] - 1: index_x = self.map.shape[0] - 1
#         if index_y < 1: index_y = 1
#         if index_y > self.map.shape[1] - 1: index_y = self.map.shape[1] - 1

#         if self.map[index_x][index_y] == 1:
#             self.map[index_x][index_y] = 2

#             self.get_logger().info("Another part reached ... percentage total reached...." + str(100 * float(np.count_nonzero(self.map == 2)) / (np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))))
#             self.get_logger().info("Discrete Map")
#             self.get_logger().info("\n" + str(self.map))

#         # Lógica de desvio de obstáculos e cobertura
#         msg = Twist()

#         if self.escape_mode:
#             # Modo de escape para sair de cantos
#             self.escape_counter += 1
#             if self.escape_counter < 10:
#                 msg.linear.x = -0.1  # Dá ré para tentar se afastar do canto
#                 msg.angular.z = 0.0
#             elif self.escape_counter < 20:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.5  # Gira para tentar sair do canto
#             else:
#                 self.escape_counter = 0
#                 self.escape_mode = False
#         elif self.obstacle_detected:
#             self.obstacle_avoidance_mode = True

#         if self.obstacle_avoidance_mode:
#             self.obstacle_avoidance_counter += 1

#             if self.front_sector < 0.3:
#                 # Obstáculo à frente
#                 if self.left_sector < self.right_sector:
#                     msg.angular.z = -0.5  # Gira para a direita
#                 else:
#                     msg.angular.z = 0.5  # Gira para a esquerda
#             elif self.left_sector < 0.3:
#                 # Obstáculo à esquerda
#                 msg.angular.z = -0.5  # Gira para a direita
#             elif self.right_sector < 0.3:
#                 # Obstáculo à direita
#                 msg.angular.z = 0.5  # Gira para a esquerda
#             else:
#                 # Sem obstáculos próximos
#                 msg.linear.x = 0.1
#                 msg.angular.z = 0.0

#             if self.obstacle_avoidance_counter >= 20:
#                 self.obstacle_avoidance_counter = 0
#                 self.obstacle_avoidance_mode = False

#         else:
#             # Caso contrário, continue a cobertura
#             msg.linear.x = 0.1
#             msg.angular.z = 0.0  # Segue em frente

#         # Se estiver girando muito ou preso em um canto, ative o modo de escape
#         if self.obstacle_avoidance_mode and self.obstacle_avoidance_counter >= 20:
#             self.escape_mode = True
#             self.obstacle_avoidance_mode = False
#             self.escape_counter = 0

#         self.publish_cmd_vel.publish(msg)

#     def callback_laser(self, msg):
#         self.laser = msg

#         # Dividindo o LIDAR em setores
#         self.front_sector = min(min(self.laser.ranges[0:30]), min(self.laser.ranges[-30:]))
#         self.left_sector = min(self.laser.ranges[60:120])
#         self.right_sector = min(self.laser.ranges[-120:-60])

#         # Verificando se há obstáculos nos setores com uma distância segura de 0.3 metros
#         if self.front_sector < 0.3 or self.left_sector < 0.3 or self.right_sector < 0.3:
#             self.obstacle_detected = True
#         else:
#             self.obstacle_detected = False

#     def callback_odom(self, msg):
#         self.odom = msg

# def main(args=None):
#     rclpy.init(args=args)
#     node = TurtlebotCtrl()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

'''
# #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random
import time  # Para usar time.sleep

from scipy.spatial import Voronoi

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")

        self.laser = LaserScan()
        self.odom = Odometry()

        self.map = np.array([   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
        self.timer = self.create_timer(0.5, self.cmd_vel_pub)

        # Encontrar pontos no mapa para criar regiões de Voronoi
        self.voronoi_points = np.argwhere(self.map == 1)
        if len(self.voronoi_points) > 0:
            self.vor = Voronoi(self.voronoi_points)
        else:
            self.vor = None

        self.prev_position = None
        self.loop_counter = 0
        self.dead_end = False
        self.retry_counter = 0  # Contador para verificar se o robô está preso
        self.safe_distance = 0.3  # Distância segura dos obstáculos

    def find_nearest_point(self, current_pos, points):
        # Calcular a distância euclidiana para todos os pontos
        distances = np.sqrt((points[:, 0] - current_pos[0])**2 + (points[:, 1] - current_pos[1])**2)
        # Encontrar o índice do ponto mais próximo
        nearest_index = np.argmin(distances)
        return points[nearest_index]

    def is_point_in_polygon(self, x, y, poly):
        # Testar se um ponto (x, y) está dentro de um polígono definido pelos vértices `poly`
        n = len(poly)
        inside = False
        p1x, p1y = poly[0]
        for i in range(n + 1):
            p2x, p2y = poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def cmd_vel_pub(self):
        map_resolution = 4

        # Atualizar a posição do robô
        index_x = -int(self.odom.pose.pose.position.x * map_resolution)
        index_y = -int(self.odom.pose.pose.position.y * map_resolution)

        index_x += int(self.map.shape[0] / 2)
        index_y += int(self.map.shape[0] / 2)

        if index_x < 1: index_x = 1
        if index_x > self.map.shape[0] - 1: index_x = self.map.shape[0] - 1
        if index_y < 1: index_y = 1
        if index_y > self.map.shape[0] - 1: index_y = self.map.shape[0] - 1

        if self.map[index_x][index_y] == 1:
            self.map[index_x][index_y] = 2

            self.get_logger().info("Another part reached ... percentage total reached...." +
                                   str(100 * float(np.count_nonzero(self.map == 2)) / (np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))))
            self.get_logger().info("Discrete Map")
            self.get_logger().info("\n" + str(self.map))

        # Encontrar todos os pontos marcados como 1
        points = np.argwhere(self.map == 1)

        if points.size > 0:
            # Posição atual do robô
            current_pos = np.array([index_x, index_y])

            # Verificar se o robô está preso
            if self.prev_position is not None and np.allclose(current_pos, self.prev_position, atol=0.1):
                self.retry_counter += 1
                if self.retry_counter > 5:
                    self.get_logger().warn("Parece que o robô está preso. Tentando recuar e redirecionar.")
                    msg = Twist()
                    msg.linear.x = -0.2  # Recuar
                    msg.angular.z = random.choice([-1, 1]) * 0.5  # Girar aleatoriamente
                    self.publish_cmd_vel.publish(msg)
                    time.sleep(2)
                    self.retry_counter = 0
                    return
            else:
                self.retry_counter = 0

            # Encontrar a região de Voronoi que o robô está atualmente
            if self.vor:
                for region_index in range(len(self.vor.point_region)):
                    vertices = self.vor.regions[self.vor.point_region[region_index]]
                    if not -1 in vertices:  # Ignorar regiões infinitas
                        polygon = [self.vor.vertices[i] for i in vertices]
                        if self.is_point_in_polygon(current_pos[0], current_pos[1], polygon):
                            # Encontre o ponto mais próximo dentro da região de Voronoi
                            points_in_region = [p for p in points if self.is_point_in_polygon(p[0], p[1], polygon)]
                            if points_in_region:
                                nearest_point = self.find_nearest_point(current_pos, np.array(points_in_region))
                                break
                else:
                    # Se não estiver em nenhuma região, encontre o ponto mais próximo geral
                    nearest_point = self.find_nearest_point(current_pos, points)
            else:
                nearest_point = self.find_nearest_point(current_pos, points)

            # Calcular o ângulo para o ponto mais próximo
            distance_x = nearest_point[0] - current_pos[0]
            distance_y = nearest_point[1] - current_pos[1]
            angle_to_nearest = np.arctan2(distance_y, distance_x)
            distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

            # Criar mensagem de comando
            msg = Twist()

            # Lógica de navegação
            obstacle_right = min(self.laser.ranges[:30])  # Lidar right side
            obstacle_left = min(self.laser.ranges[-30:])  # Lidar left side
            obstacle_front = min(min(self.laser.ranges[330:360]), min(self.laser.ranges[0:30]))  # Lidar front

            if obstacle_front < self.safe_distance:
                self.get_logger().info("Obstáculo à frente, recuando...")
                msg.linear.x = -0.2  # Recuar se obstáculo estiver muito perto à frente
                msg.angular.z = random.choice([-1, 1]) * 0.5  # Girar aleatoriamente
            elif obstacle_right < self.safe_distance:
                self.get_logger().info("Obstáculo à direita, girando à esquerda...")
                msg.linear.x = 0.0
                msg.angular.z = 0.5  # Girar para a esquerda
            elif obstacle_left < self.safe_distance:
                self.get_logger().info("Obstáculo à esquerda, girando à direita...")
                msg.linear.x = 0.0
                msg.angular.z = -0.5  # Girar para a direita
            else:
                if distance > 0.1:
                    msg.linear.x = min(0.2 * distance, 0.2)
                    msg.angular.z = 0.15 * angle_to_nearest  # Reduzir o ganho angular
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0

            self.publish_cmd_vel.publish(msg)

            # Detecção de beco sem saída
            if obstacle_front < self.safe_distance and obstacle_right < self.safe_distance and obstacle_left < self.safe_distance:
                self.get_logger().warn("Beco sem saída detectado. Recuo...")
                self.dead_end = True
                msg.linear.x = -0.2  # Recuar
                msg.angular.z = 0.0
                self.publish_cmd_vel.publish(msg)
                time.sleep(2)  # Esperar um pouco antes de tentar outro caminho

            self.prev_position = current_pos

    def callback_laser(self, msg):
        self.laser = msg

    def callback_odom(self, msg):
        self.odom = msg

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
# import numpy as np

# class TurtlebotCtrl(Node):
#     def __init__(self):
#         super().__init__("TurtlebotCtrl")

#         self.laser = LaserScan()
#         self.odom = Odometry()

#         self.map = np.array([	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
#                                 [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
#                                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#                             ])

#         self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
#         self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
#         self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
#         self.timer = self.create_timer(0.5, self.cmd_vel_pub)

#         self.state = 'MOVE_TO_GOAL'  # Estado inicial
#         self.wall_following_direction = None

#     def cmd_vel_pub(self):
#         map_resolution = 4

#         index_x = -int(self.odom.pose.pose.position.x * map_resolution)
#         index_y = -int(self.odom.pose.pose.position.y * map_resolution)

#         index_x += int(self.map.shape[0] / 2)
#         index_y += int(self.map.shape[1] / 2)

#         if (index_x < 1): index_x = 1
#         if (index_x > self.map.shape[0] - 1): index_x = self.map.shape[0] - 1
#         if (index_y < 1): index_y = 1
#         if (index_y > self.map.shape[1] - 1): index_y = self.map.shape[1] - 1

#         if (self.map[index_x][index_y] == 1):
#             self.map[index_x][index_y] = 2

#             self.get_logger().info("Another part reached ... percentage total reached...." +
#                                    str(100 * float(np.count_nonzero(self.map == 2)) /
#                                        (np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))))
#             self.get_logger().info("Discrete Map")
#             self.get_logger().info("\n" + str(self.map))

#         # Encontrar o próximo ponto não percorrido mais próximo
#         next_point = None
#         min_distance = float('inf')
#         for i in range(self.map.shape[0]):
#             for j in range(self.map.shape[1]):
#                 if self.map[i][j] == 1:
#                     distance = np.sqrt((i - index_x)**2 + (j - index_y)**2)
#                     if distance < min_distance:
#                         min_distance = distance
#                         next_point = (i, j)

#         if next_point is not None:
#             target_x, target_y = next_point
#             direction_vector = np.array([target_x - index_x, target_y - index_y])
#             angle_to_target = np.arctan2(direction_vector[1], direction_vector[0])

#             # Calcular a diferença angular em relação à direção atual do robô
#             current_orientation = self.get_yaw_from_quaternion(self.odom.pose.pose.orientation)
#             angle_difference = self.normalize_angle(angle_to_target - current_orientation)

#             min_distance_to_obstacle = min(self.laser.ranges)
#             safe_distance = 0.3  # Distância segura dos obstáculos

#             msg = Twist()

#             if self.state == 'MOVE_TO_GOAL':
#                 if min_distance_to_obstacle < safe_distance:
#                     self.state = 'FOLLOW_WALL'
#                     # Determinar a direção inicial para seguir a parede (direita ou esquerda)
#                     self.wall_following_direction = self.determine_wall_following_direction()
#                 elif abs(angle_difference) > 0.1:
#                     msg.angular.z = 0.3 if angle_difference > 0 else -0.3
#                     msg.linear.x = 0.0
#                 else:
#                     msg.linear.x = 0.15
#                     msg.angular.z = 0.0

#             elif self.state == 'FOLLOW_WALL':
#                 if min_distance_to_obstacle >= safe_distance:
#                     self.state = 'MOVE_TO_GOAL'
#                 else:
#                     # Seguir a parede mantendo uma distância segura
#                     self.wall_following(msg)

#             self.publish_cmd_vel.publish(msg)

#     def determine_wall_following_direction(self):
#         """
#         Determina se o robô deve seguir a parede pela direita ou esquerda.
#         """
#         front_left = min(self.laser.ranges[45:90])  # LIDAR direção frontal esquerda
#         front_right = min(self.laser.ranges[270:315])  # LIDAR direção frontal direita

#         return 'LEFT' if front_left < front_right else 'RIGHT'

#     def wall_following(self, msg):
#         """
#         Função que implementa o comportamento de seguir a parede.
#         """
#         if self.wall_following_direction == 'LEFT':
#             msg.angular.z = 0.3  # Girar para a esquerda
#             msg.linear.x = 0.1  # Avançar lentamente
#         else:
#             msg.angular.z = -0.3  # Girar para a direita
#             msg.linear.x = 0.1  # Avançar lentamente

#     def get_yaw_from_quaternion(self, q):
#         """
#         Converte a orientação em quaternion para yaw (ângulo em radianos).
#         """
#         import math
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     def normalize_angle(self, angle):
#         """
#         Normaliza o ângulo para o intervalo [-pi, pi].
#         """
#         import math
#         while angle > math.pi:
#             angle -= 2 * math.pi
#         while angle < -math.pi:
#             angle += 2 * math.pi
#         return angle

#     def callback_laser(self, msg):
#         self.laser = msg

#     def callback_odom(self, msg):
#         self.odom = msg

# def main(args=None):
#     rclpy.init(args=args)
#     node = TurtlebotCtrl()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3


