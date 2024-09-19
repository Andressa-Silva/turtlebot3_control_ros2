#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class TurtlebotCtrl(Node):
    def __init__(self):
        super().__init__("TurtlebotCtrl")
        
        self.laser = LaserScan()
        self.odom  = Odometry()
        
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
        
        self.publish_cmd_vel  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_odom  = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
        self.timer = self.create_timer(0.5, self.cmd_vel_pub)
        self.map_resolution = 4
        
    def cmd_vel_pub(self):     
        for i in range(self.map.shape[0]):  # Percorre as linhas
            for j in range(self.map.shape[1]):  # Percorre as colunas
                valor = self.map[i, j]
                
                # Calcula a posição atual do robô como índices na matriz
                index_x = -int(self.odom.pose.pose.position.x * self.map_resolution)
                index_y = -int(self.odom.pose.pose.position.y * self.map_resolution)

                # Ajusta para o centro da matriz
                index_x += int(self.map.shape[0]/2)
                index_y += int(self.map.shape[0]/2)

                # Garante que os índices estejam dentro dos limites da matriz
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
                    
            
                if valor != 0:
                    # print(f"Matriz [{i}, {j}] = {valor}")
                    
                    cord_x,cord_y = self.convert_to_physical_position(i,j)
                    # print('poseeeee',cord_x,cord_y)
                    
                    self.publish_next_target(cord_x,cord_y)
                    
                else:
                    pass
                
    def convert_to_physical_position(self, index_x, index_y):
       
        # print("aaaaaaaa",index_x,index_y)
        x_real = (index_x )*self.map_resolution
        y_real = (index_y )*self.map_resolution
        return x_real, y_real

    def publish_next_target(self, target_x, target_y):
        # Obtém a posição atual do robô
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y

        # Calcula o erro em x e y
        delta_x = target_x - current_x
        delta_y = target_y - current_y

        # Calcula a distância até o alvo
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        # Calcula o ângulo para o alvo
        target_angle = np.arctan2(delta_y, delta_x)
        current_angle = self.odom.pose.pose.orientation.z

        # Calcula o erro de orientação
        angle_error = target_angle - current_angle

        # Cria uma mensagem Twist para enviar comandos de movimento
        msg = Twist()

        # Adiciona o desvio de obstáculo baseado no sensor LIDAR
        obstacle_right = self.laser.ranges[:30]  # Faixa de leitura para a direita
        obstacle_left = self.laser.ranges[-30:]  # Faixa de leitura para a esquerda

        # Verifica obstáculos e ajusta o comportamento
        if min(obstacle_right) < np.random.uniform(0.3, 0.5):  # Obstáculo à direita
            msg.linear.x = 0.0
            msg.angular.z = -0.4  # Gira para a esquerda
        elif min(obstacle_left) < np.random.uniform(0.3, 0.5):  # Obstáculo à esquerda
            msg.linear.x = 0.0
            msg.angular.z = 0.4  # Gira para a direita
        else:
            # Se não houver obstáculos, segue para o próximo alvo
            if distance > 0.1:
                msg.linear.x = min(0.3 * distance, 0.3)  # Limita a velocidade linear
                msg.angular.z = 0.3 * angle_error  # Ajusta a velocidade angular
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        # Publica a mensagem de comando de movimento
        self.publish_cmd_vel.publish(msg)

        # Verifica se o robô chegou perto o suficiente do alvo
        if distance <= 0.1:
            self.get_logger().info(f"Alvo alcançado: ({target_x}, {target_y})")
            self.current_target = None  # Reseta o alvo atual
            

    def callback_laser(self, msg):
        self.laser = msg

    def callback_odom(self, msg):
        self.odom = msg

def main(args = None):
	rclpy.init(args = args)
	node = TurtlebotCtrl()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
