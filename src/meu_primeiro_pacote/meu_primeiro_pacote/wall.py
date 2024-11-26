import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Comandos de movimento
        self.ir_para_frente = Twist(linear=Vector3(x=0.2, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.virar_direita = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.virar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.distancia_desejada_da_parede = 1.0  # Distância desejada da parede
        self.distancia_limite_frente = 1.0  # Distância limite para virar à esquerda quando houver obstáculo na frente

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def seguir_parede(self):
        if self.laser:
            # Distâncias medidas pelos sensores de laser
            distancia_direita = min(self.laser[0:80])  # Lado direito (entre -90° e -10°)
            distancia_frente = min(self.laser[80:100])  # Frente (entre -10° e 10°)
            distancia_esquerda = min(self.laser[100:180])  # Lado esquerdo (entre 10° e 90°)

            self.get_logger().info(f'Distâncias - Direita: {distancia_direita}, Frente: {distancia_frente}, Esquerda: {distancia_esquerda}')

            # Se houver um obstáculo à frente, gira à esquerda
            if distancia_frente < self.distancia_limite_frente:
                self.get_logger().info('Obstáculo à frente! Virando à esquerda...')
                self.pub_cmd_vel.publish(self.virar_esquerda)
                return  # Sai da função após o comando para evitar sobreposição

            # Se estiver muito longe da parede à direita, gira à direita para se aproximar
            if distancia_direita > self.distancia_desejada_da_parede + 0.2:
                self.get_logger().info('Muito longe da parede à direita. Virando à direita...')
                self.pub_cmd_vel.publish(self.virar_direita)
            # Se estiver muito perto da parede à direita, gira à esquerda para se afastar
            elif distancia_direita < self.distancia_desejada_da_parede - 0.2:
                self.get_logger().info('Muito perto da parede à direita. Virando à esquerda...')
                self.pub_cmd_vel.publish(self.virar_esquerda)
            # Se a distância estiver adequada, continua andando para frente
            else:
                self.get_logger().info('Distância correta da parede. Seguindo em frente...')
                self.pub_cmd_vel.publish(self.ir_para_frente)

    def run(self):
        self.get_logger().info('Iniciando o comportamento de seguir parede...')
        while rclpy.ok():
            rclpy.spin_once(self)
            self.seguir_parede()

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
