import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

# Variáveis globais
MAX_DIST = 2.0  # Distância máxima para considerar obstáculos
mapa = np.zeros((10, 10))  
robo_pos = (-9, -9)  # Posição inicial do robô
objetivo_pos = (9, 9)  # Posição do objetivo
mapa[objetivo_pos] = 2  # Marca o objetivo no mapa

def converter_laser_para_mapa(angulo, distancia):
    x = int(robo_pos[0] + distancia * np.cos(np.radians(angulo)))
    y = int(robo_pos[1] + distancia * np.sin(np.radians(angulo)))
    return x, y

# Algoritmo Wavefront
def wavefront(mapa, objetivo):
    fila = [objetivo]
    while fila:
        x, y = fila.pop(0)
        valor_atual = mapa[x, y]
        vizinhos = [(x+dx, y+dy) for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]]
        for vx, vy in vizinhos:
            if 0 <= vx < mapa.shape[0] and 0 <= vy < mapa.shape[1] and mapa[vx, vy] == 0:
                mapa[vx, vy] = valor_atual + 1
                fila.append((vx, vy))
    return mapa

# Encontra o menor caminho
def encontrar_caminho(mapa, robo):
    caminho = []
    posicao_atual = robo
    while mapa[posicao_atual] != 2:
        caminho.append(posicao_atual)
        x, y = posicao_atual
        vizinhos = [(x+dx, y+dy) for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]]
        posicao_atual = min(vizinhos, key=lambda pos: mapa[pos] if 0 <= pos[0] < mapa.shape[0] and 0 <= pos[1] < mapa.shape[1] else float('inf'))
    caminho.append(posicao_atual)  # Adiciona o objetivo
    return caminho

class LaserScanNode(Node):

    def __init__(self):
        super().__init__('scan_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Nó LaserScan iniciado.')

    # Callback para lidar com os dados do LaserScan
    def scan_callback(self, msg):
        global mapa, robo_pos
        distancias = msg.ranges
        for i, distancia in enumerate(distancias):
            if distancia < MAX_DIST:  # Limite para considerar um obstáculo
                obst_x, obst_y = converter_laser_para_mapa(i, distancia)
                if 0 <= obst_x < mapa.shape[0] and 0 <= obst_y < mapa.shape[1]:
                    mapa[obst_x, obst_y] = 1  # Marcar como obstáculo no mapa
        
        # Após processar os dados do laser, executar o algoritmo Wavefront
        mapa_wavefront = wavefront(mapa, objetivo_pos)
        
        caminho = encontrar_caminho(mapa_wavefront, robo_pos)
        
        self.get_logger().info(f'Caminho encontrado: {caminho}')

def main(args=None):
    rclpy.init(args=args)

    scan_node = LaserScanNode()

    rclpy.spin(scan_node)

    scan_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
