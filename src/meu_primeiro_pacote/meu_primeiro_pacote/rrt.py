import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import math

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        qos_profile = QoSProfile(depth=10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.listener_callback_scan, qos_profile)

        # Lista de waypoints do mapa 2D
        original_waypoints = [
            (500, 500), (487.2230660962495, 492.1418857211675), (481.12353460118766, 478.4380280328832),
            (469.0761787108363, 469.501517749153), (454.18553696698916, 471.30949685754894), (443.8140175788205, 460.472911344536),
            (434.86703398046205, 448.4333314311542), (431.588134179264, 433.79609070437436), (418.1717681493445, 427.08780310236875),
            (405.84780072690734, 435.63881631547446), (390.94608111933246, 437.3530953876263), (380.390837311318, 426.6953832432037),
            (377.08180089868284, 412.0649260384594), (366.68642327385606, 401.2512252826011), (369.6521423801828, 386.5473308942981),
            (375.2595233285996, 372.6348444558741), (362.6006351773328, 364.58794749035275), (353.91579021537467, 352.35792426933017),
            (360.14743239274407, 338.71363477616745), (350.8983240833958, 326.90457065228594), (337.288682652802, 320.59761733624936),
            (323.4588268746398, 314.7894257799013), (317.13191271490126, 301.1890523579948), (303.15368174861305, 295.7476316746529),
            (288.7822294851519, 291.4509632165267), (278.4605846973457, 280.5668624785533), (269.5486814557425, 268.5012924293471),
            (255.1597287126358, 264.2635997839748), (241.7551364562794, 270.9953830871442), (228.32914783836566, 277.6843912073105),
            (216.13202062595343, 268.95340685477424), (206.9664729309105, 257.07937066441787), (197.34042456598831, 245.5754933669073),
            (182.3521670354492, 244.98208271799896), (167.7888758721726, 241.38895726880716), (159.93015940048946, 228.61239374735524),
            (154.20351214130065, 214.74857352460188), (142.23615934740883, 205.70520760772501), (133.67918877711517, 193.3853758451104),
            (125.6144213143005, 180.737865190016), (114.30386828910828, 170.88538376327304), (112.95891257073805, 155.94580230609617),
            (100.09412235271263, 148.23236126812893), (89.21590620390798, 137.90451476365928), (84.06093244143533, 123.81812965220436),
            (69.1848645514371, 125.74234485123068), (54.21501759846992, 124.79172284183933), (53.0, 114.0), (50, 100)
        ]

        # Converte os waypoints para o sistema do robô
        self.waypoints = self.convert_waypoints(original_waypoints)
        self.current_waypoint_index = 0

        # Variáveis de estado
        self.current_position = None
        self.current_orientation = None
        self.obstacle_detected = False  # Indica se há obstáculo à frente
        self.side_distance = None  # Distância lateral da parede

    def convert_waypoints(self, waypoints):
        # Converte coordenadas do sistema original para o sistema do robô
        new_waypoints = []
        for x, y in waypoints:
            new_x = -9 + (x - 500) * (9 - (-9)) / (50 - 500)
            new_y = -9 + (y - 500) * (7 - (-9)) / (100 - 500)
            new_waypoints.append((new_x, new_y))
        return new_waypoints

    def listener_callback_odom(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def listener_callback_scan(self, msg):
        # Verifica se há obstáculos à frente
        front_range = msg.ranges[len(msg.ranges) // 2]
        self.obstacle_detected = front_range < 1.0 

        # Captura a distância lateral (para seguir a parede)
        self.side_distance = min(msg.ranges[:len(msg.ranges)//4])  # Lado direito

    def navigate_to_waypoints(self):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info('Esperando dados de odometria...')
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Todos os waypoints alcançados! Parando o robô...')
            self.pub_cmd_vel.publish(Twist())  # Envia comando de parada
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        current_x, current_y = self.current_position
        dx, dy = target_x - current_x, target_y - current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        if distance_to_target <= 1:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} alcançado: ({target_x}, {target_y})')
            self.current_waypoint_index += 1
            return

        if self.obstacle_detected:
            self.follow_wall()
        else:
            self.navigate_towards_waypoint(dx, dy)

    def navigate_towards_waypoint(self, dx, dy):
        target_theta = math.atan2(dy, dx)
        error_theta = target_theta - self.current_orientation
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

        angular_speed = 0.5 * error_theta
        linear_speed = 0.2 if abs(error_theta) < 0.1 else 0.0

        twist = Twist(
            linear=Vector3(x=linear_speed, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_speed)
        )
        self.pub_cmd_vel.publish(twist)

    def follow_wall(self):
        # Lógica para seguir a parede
        if self.side_distance is not None:
            error = self.side_distance - 0.5  # Tenta manter 0.5m da parede
            angular_speed = -0.5 * error
            linear_speed = 0.1

            twist = Twist(
                linear=Vector3(x=linear_speed, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=angular_speed)
            )
            self.pub_cmd_vel.publish(twist)

    def run(self):
        self.get_logger().info('Iniciando navegação para os waypoints...')
        while rclpy.ok():
            rclpy.spin_once(self)
            self.navigate_to_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
