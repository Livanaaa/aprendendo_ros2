import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug('Definindo o publisher de controle do robo: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Definindo os comandos de movimento
        self.ir_para_frente = Twist(linear=Vector3(x=0.2, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.virar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))  # Virar a 0.5 rad/s

        self.movendo_para_frente = True  # Controle do estado de movimento

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def girar_90_graus_para_esquerda(self):
        # Publica o comando para girar o robô 90° para a esquerda
        self.get_logger().info('Ordenando o robô: "girar 90 graus para a esquerda"')
        tempo_giro = 3.2  # Aproximado para 90 graus a 0.5 rad/s

        # Inicia o giro
        self.pub_cmd_vel.publish(self.virar_esquerda)
        rclpy.spin_once(self)

        # Espera o tempo necessário para completar o giro
        self.get_logger().info(f'Girando por {tempo_giro} segundos...')
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=tempo_giro))

        # Para de girar
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)

        # Volta para o modo de movimento para frente
        self.movendo_para_frente = True

    def run(self):
        self.get_logger().debug('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().info('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        self.get_logger().info('Entrando no loop principal do nó.')
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.laser:
                self.get_logger().debug('Atualizando as distâncias lidas pelo laser.')
                self.distancia_frente = min(self.laser[80:100])  # -10 a 10 graus para frente

                self.get_logger().debug('Distância para o obstáculo à frente: ' + str(self.distancia_frente))
                if self.distancia_frente < 1.5:  # Se estiver muito perto de uma parede
                    self.get_logger().info('Obstáculo detectado. Parando e virando...')
                    self.pub_cmd_vel.publish(self.parar)  # Para o robô
                    rclpy.spin_once(self)

                    # Gira 90 graus para a esquerda
                    self.movendo_para_frente = False  # Alterando o estado para girar
                    self.girar_90_graus_para_esquerda()

                # Se não há obstáculo, e o robô deveria estar andando para frente
                elif self.movendo_para_frente:
                    self.get_logger().info('Nenhum obstáculo à frente, continuando a andar...')
                    self.pub_cmd_vel.publish(self.ir_para_frente)
                    rclpy.spin_once(self)

    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
