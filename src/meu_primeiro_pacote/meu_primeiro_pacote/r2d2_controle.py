import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np  # array operations
import tf_transformations  # to handle quaternions
import math  # for angle and distance calculations

class R2D2(Node):
    def __init__(self):
        super().__init__('R2D2')

        # Definir a coordenada objetivo (9, 9, 0)
        self.goal_x = 9.0
        self.goal_y = 9.0

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber da odometria: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug('Definindo o publisher de controle do robô: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        rclpy.spin_once(self)

        # Inicialize o ganho proporcional
        self.p_gain = 0.5  # Ajustar o ganho proporcional para rotação
        self.linear_speed = 0.5  # Velocidade linear do robô

        # Pré-definir comandos de giro à direita e à esquerda
        self.girar_direita = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=-0.5, z=0.0))
        self.girar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.5, z=0.0))

        # Distância mínima para um obstáculo ser considerado próximo
        self.distancia_obstaculo = 0.2  # Ajustado para 0.2 metros

        # Inicializando variáveis de controle
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.estado_atual = 'girar'

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def get_yaw_from_quaternion(self):
        orientation_q = self.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        return yaw

    def calculate_distance_to_goal(self):
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        distance = math.sqrt((self.goal_x - current_x) ** 2 + (self.goal_y - current_y) ** 2)
        return distance

    def calculate_angle_to_goal(self):
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        goal_angle = math.atan2(self.goal_y - current_y, self.goal_x - current_x)
        return goal_angle

    def find_best_laser_direction(self):
        # Encontrar a direção com a maior distância usando os dados do laser
        if self.laser is None:
            return None, None

        # Encontrar o índice com a maior distância no array do laser
        max_distance = max(self.laser)
        max_index = self.laser.index(max_distance)

        # Converter o índice para um ângulo relativo
        angle = (max_index - len(self.laser) / 2) * (math.pi / len(self.laser))  # ângulo em radianos
        return angle, max_distance

    def run(self):
        self.get_logger().info('Entrando no loop principal do nó.')


        while rclpy.ok():
            rclpy.spin_once(self)

            self.pub_cmd_vel.publish(self.ir_para_frente)
            continue
            
            if self.laser is not None and self.pose is not None:
                # 1. Obtenha a orientação atual do robô (Yaw)
                yaw_atual = self.get_yaw_from_quaternion()

                # 2. Calcule a distância e o ângulo para o objetivo (9, 9)
                distancia_ate_meta = self.calculate_distance_to_goal()
                angulo_para_meta = self.calculate_angle_to_goal()

                # 3. Se a distância até a meta for pequena o suficiente, parar o robô
                if distancia_ate_meta < 0.5:
                    self.get_logger().info('Objetivo alcançado! Parando o robô.')
                    self.pub_cmd_vel.publish(self.parar)
                    rclpy.spin_once(self)

                    break

                # Verificar estado do robô
                if self.estado_atual == 'girar':
                    # 4. Verificar o erro angular e alinhar-se com o objetivo
                    erro_angulo = angulo_para_meta - yaw_atual
                    erro_angulo = np.arctan2(np.sin(erro_angulo), np.cos(erro_angulo))  # Normaliza o erro angular

                    if abs(erro_angulo) > 0.1:
                        # Chama o comando de giro apropriado
                        self.controlar_motores_giro(erro_angulo)
                    else:
                        # Se já estiver alinhado, mudar para o estado de andar
                        self.estado_atual = 'andar'
                        self.get_logger().info("Alinhado com o objetivo. Mudando para o estado 'andar'.")

                elif self.estado_atual == 'andar':
                    # 5. Verificar se o caminho para o objetivo está livre de obstáculos
                    angle_to_goal = angulo_para_meta - yaw_atual
                    laser_index = int((angle_to_goal + math.pi) / (2 * math.pi) * len(self.laser))
                    laser_distance_to_goal = self.laser[laser_index]

                    if laser_distance_to_goal > self.distancia_obstaculo:
                        # Se a linha para o objetivo estiver livre, mover-se para frente
                        self.get_logger().info(f"Movendo-se em direção ao objetivo. Distância restante: {distancia_ate_meta}")
                        self.pub_cmd_vel.publish(self.ir_para_frente)
                        rclpy.spin_once(self)
                    else:
                        # Se o caminho para o objetivo estiver bloqueado, encontrar a melhor direção usando o laser
                        melhor_angulo, melhor_distancia = self.find_best_laser_direction()
                        if melhor_angulo is not None:
                            self.get_logger().info(f"Direção mais livre encontrada. Ângulo: {melhor_angulo}, Distância: {melhor_distancia}")
                            self.estado_atual = 'girar'  # Voltar para o estado de girar e desviar

            else:
                self.get_logger().debug('Aguardando dados de laser e odometria.')

        self.get_logger().info('Ordenando o robô: "parar"')
        #self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)

    # Função para controlar os motores durante o giro
    def controlar_motores_giro(self, erro_angular):
        if erro_angular > 0:
            # Girar à esquerda
            self.get_logger().info(f"Girando à esquerda. Erro de ângulo: {erro_angular}")
            self.pub_cmd_vel.publish(self.girar_esquerda)
            rclpy.spin_once(self)
        else:
            # Girar à direita
            self.get_logger().info(f"Girando à direita. Erro de ângulo: {erro_angular}")
            self.pub_cmd_vel.publish(self.girar_direita)
            rclpy.spin_once(self)

    # Função para destruir o nó de forma segura
    def destroy(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')
        self.destroy_node()
        rclpy.shutdown()

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()

if __name__ == '__main__':
    main()
