import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- CONFIGURAÇÃO ---
        self.stop_distance = 0.35      # Distância de parada frontal/traseira
        self.side_stop_distance = 0.30 # Distância de segurança lateral (pode ser menor)

        # --- SUBSCRIBERS (Ouvindo os 4 sensores) ---
        self.create_subscription(Range, '/ultrasound/front', self.front_callback, 10)
        self.create_subscription(Range, '/ultrasound/rear',  self.rear_callback, 10)
        self.create_subscription(Range, '/ultrasound/left',  self.left_callback, 10)
        self.create_subscription(Range, '/ultrasound/right', self.right_callback, 10)

        # Ouve o comando do Joystick/RQT
        self.create_subscription(Twist, '/cmd_vel_teleop', self.cmd_callback, 10)

        # --- PUBLISHER ---
        # Envia o comando "limpo" para o Mux
        self.filtered_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        # Inicializa distâncias como "infinito" (9.9m)
        self.dist_front = 9.9
        self.dist_rear = 9.9
        self.dist_left = 9.9
        self.dist_right = 9.9

    # --- CALLBACKS DOS SENSORES ---
    def front_callback(self, msg): self.dist_front = msg.range
    def rear_callback(self, msg):  self.dist_rear = msg.range
    def left_callback(self, msg):  self.dist_left = msg.range
    def right_callback(self, msg): self.dist_right = msg.range

    # --- LÓGICA DE FILTRAGEM ---
    def cmd_callback(self, msg):
        filtered_cmd = Twist()
        
        # Copia o comando original inicialmente
        filtered_cmd.linear.x = msg.linear.x
        filtered_cmd.angular.z = msg.angular.z

        # 1. SEGURANÇA LINEAR (Frente e Trás)
        # Lógica invertida para corrigir orientação
        if msg.linear.x > 0 and self.dist_front < self.stop_distance:
            filtered_cmd.linear.x = 0.0
            self.get_logger().warn(f'Bloqueando RÉ! Obs a {self.dist_front:.2f}m', throttle_duration_sec=1)
        
        # Lógica invertida para corrigir orientação
        elif msg.linear.x < 0 and self.dist_rear < self.stop_distance:
            filtered_cmd.linear.x = 0.0
            self.get_logger().warn(f'Bloqueando FRENTE! Obs a {self.dist_rear:.2f}m', throttle_duration_sec=1)

        # 2. SEGURANÇA ANGULAR (Laterais)
        
        # Se quer girar para ESQUERDA (Z > 0) e tem obstáculo na esquerda
        if msg.angular.z > 0 and self.dist_left < self.side_stop_distance:
            filtered_cmd.angular.z = 0.0
            self.get_logger().warn(f'Bloqueando GIRO ESQ! Parede a {self.dist_left:.2f}m', throttle_duration_sec=1)

        # Se quer girar para DIREITA (Z < 0) e tem obstáculo na direita
        elif msg.angular.z < 0 and self.dist_right < self.side_stop_distance:
            filtered_cmd.angular.z = 0.0
            self.get_logger().warn(f'Bloqueando GIRO DIR! Parede a {self.dist_right:.2f}m', throttle_duration_sec=1)

        # Publica o comando filtrado
        self.filtered_pub.publish(filtered_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
