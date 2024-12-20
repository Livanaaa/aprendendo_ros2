import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt


class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')

        # Configuração de publishers e subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, 10)

        # Carregar o mapa
        self.map_image = cv2.imread('src/meu_primeiro_pacote/meu_primeiro_pacote/my_map].pgm', cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            self.get_logger().error("Erro ao carregar o mapa. Verifique o caminho do arquivo.")
            exit()

        # Binarizar o mapa: Áreas brancas (>200) são navegáveis, pretas (<200) são obstáculos
        self.map_binary = 1.0 * (self.map_image >= 200)

        # Determinar limites do mapa navegável
        self.valid_area = np.argwhere(self.map_binary == 1.0)
        self.min_x, self.min_y = self.valid_area.min(axis=0)
        self.max_x, self.max_y = self.valid_area.max(axis=0)

        # Definições de pontos e configurações do RRT
        self.goal = (60, 268)  # Coordenadas do objetivo
        self.robot_position = (366, 366)  # Posição inicial do robô
        self.factor_growth = 20  # Tamanho do passo do RRT
        self.max_iterations = 5000  # Número máximo de iterações

        # Estruturas para o algoritmo RRT
        self.explored_nodes = [self.robot_position]
        self.explored_paths = []
        self.parent_nodes, self.child_nodes = [], []

        # Dados de odometria
        self.current_position = None
        self.current_orientation = None

        # Executar o algoritmo RRT
        self.run_rrt_algorithm()

    def listener_callback_odom(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def run_rrt_algorithm(self):
        # Verificar se os pontos inicial e objetivo são navegáveis
        if self.map_binary[self.robot_position[0]][self.robot_position[1]] == 0:
            self.get_logger().error("Ponto inicial está em um obstáculo.")
            return
        if self.map_binary[self.goal[0]][self.goal[1]] == 0:
            self.get_logger().error("Ponto objetivo está em um obstáculo.")
            return

        for _ in range(self.max_iterations):
            # Gerar um ponto aleatório dentro dos limites válidos
            random_point = (
                np.random.randint(self.min_x, self.max_x + 1),
                np.random.randint(self.min_y, self.max_y + 1)
            )

            # Validar se o ponto aleatório está em área navegável
            if self.map_binary[random_point[0], random_point[1]] == 0:
                continue

            nearest_node = self.find_nearest_node(self.explored_nodes, random_point)
            new_node = self.generate_new_node(nearest_node, random_point, self.factor_growth)

            if self.is_collision_free(nearest_node, new_node):
                self.explored_nodes.append(new_node)
                self.explored_paths.append((nearest_node, new_node))
                self.parent_nodes.append(nearest_node)
                self.child_nodes.append(new_node)

                if math.dist(new_node, self.goal) < self.factor_growth:
                    print("Objetivo alcançado!")
                    break

        self.path = self.get_final_path()
        if self.path:
            self.display_map_with_paths(self.path)
            self.move_robot(self.path)
        else:
            self.get_logger().error("Não foi possível encontrar um caminho para o objetivo.")

    def find_nearest_node(self, explored_nodes, random_point):
        return min(explored_nodes, key=lambda node: math.dist(node, random_point))

    def generate_new_node(self, nearest_node, random_point, factor_growth):
        direction_vector = (random_point[0] - nearest_node[0], random_point[1] - nearest_node[1])
        vector_magnitude = math.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)

        if vector_magnitude < factor_growth:
            return random_point

        factor = factor_growth / vector_magnitude
        return (int(nearest_node[0] + direction_vector[0] * factor), int(nearest_node[1] + direction_vector[1] * factor))

    def is_collision_free(self, node, new_node):
        line_x = np.linspace(node[0], new_node[0], num=10, endpoint=True).astype(int)
        line_y = np.linspace(node[1], new_node[1], num=10, endpoint=True).astype(int)
        for x, y in zip(line_x, line_y):
            if x < 0 or y < 0 or x >= self.map_binary.shape[0] or y >= self.map_binary.shape[1]:
                return False
            if self.map_binary[x, y] == 0:  # Obstáculo detectado
                return False
        return True

    def get_final_path(self):
        final_path = []
        child_node = self.explored_nodes[-1]
        while child_node != self.robot_position:
            if child_node not in self.child_nodes:
                return None
            index = self.child_nodes.index(child_node)
            parent_node = self.parent_nodes[index]
            final_path.append(parent_node)
            child_node = parent_node
        final_path.reverse()
        return final_path

    def display_map_with_paths(self, final_path):
        colored_map = cv2.cvtColor(self.map_image.copy(), cv2.COLOR_GRAY2RGB)

        for (point1, point2) in self.explored_paths:
            cv2.line(colored_map, point1[::-1], point2[::-1], (0, 0, 255), 1)

        for (point1, point2) in zip(final_path[:-1], final_path[1:]):
            cv2.line(colored_map, point1[::-1], point2[::-1], (0, 255, 0), 2)

        plt.imshow(colored_map)
        plt.title("Caminho gerado pelo RRT")
        plt.show()

    def move_robot(self, path):
        for target in path:
            self.navigate_to_target(target)

    def navigate_to_target(self, target):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("Aguardando odometria...")
            return

        dx, dy = target[1] - self.current_position[0], target[0] - self.current_position[1]
        distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

        while distance_to_target > 0.5:
            target_theta = math.atan2(dy, dx)
            error_theta = target_theta - self.current_orientation
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

            twist = Twist()
            twist.linear.x = 0.2 if abs(error_theta) < 0.1 else 0.0
            twist.angular.z = 0.5 * error_theta
            self.cmd_vel_publisher.publish(twist)

            rclpy.spin_once(self)
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

        self.get_logger().info(f"Alvo {target} alcançado!")

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info("Robô parado.")


def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
