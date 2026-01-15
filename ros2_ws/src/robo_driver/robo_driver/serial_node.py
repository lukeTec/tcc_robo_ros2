import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Range  # <--- NOVA IMPORTAÇÃO
import tf2_ros
import serial
import math
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # --- Configuração Serial ---
        # Tente portas diferentes caso mude (ACM0, ACM1, etc)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Conectado ao Arduino na /dev/ttyACM0')
        except:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                self.get_logger().info('Conectado ao Arduino na /dev/ttyUSB0')
            except:
                self.get_logger().error('ERRO: Não foi possível conectar ao Arduino!')
        
        self.ser.flush()

        # --- Publishers de Odometria ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Publishers de Ultrassom (Range) ---
        self.pub_us_front = self.create_publisher(Range, 'ultrasound/front', 10)
        self.pub_us_rear  = self.create_publisher(Range, 'ultrasound/rear', 10)
        self.pub_us_left  = self.create_publisher(Range, 'ultrasound/left', 10)
        self.pub_us_right = self.create_publisher(Range, 'ultrasound/right', 10)

        # --- Subscriber (Comandos de Velocidade do MUX) ---
        # Agora ouvimos o cmd_vel final que sai do Twist Mux
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.cmd_vel_callback,
            10)

        # Timer para ler a serial
        self.timer = self.create_timer(0.01, self.read_serial_data)

        # Variáveis de Odometria
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Envia comando V,linear,angular para o Arduino
        # Ex: V,0.5,-0.1
        command = f"V,{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.write(command.encode('utf-8'))

    def read_serial_data(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # --- PARSE ODOMETRIA (Começa com O) ---
                if line.startswith('O'):
                    self.parse_odometry(line)
                
                # --- PARSE SENSORES (Começa com S) ---
                elif line.startswith('S'):
                    self.parse_sensors(line)

        except Exception as e:
            self.get_logger().warn(f'Erro na serial: {e}')

    def parse_sensors(self, line):
        # Formato: S,front,rear,left,right
        parts = line.split(',')
        if len(parts) == 5:
            try:
                # Converte cm para metros para o ROS
                d_front = float(parts[1]) / 100.0
                d_rear  = float(parts[2]) / 100.0
                d_left  = float(parts[3]) / 100.0
                d_right = float(parts[4]) / 100.0

                self.publish_range(self.pub_us_front, d_front, "us_front_link")
                self.publish_range(self.pub_us_rear,  d_rear,  "us_rear_link")
                self.publish_range(self.pub_us_left,  d_left,  "us_left_link")
                self.publish_range(self.pub_us_right, d_right, "us_right_link")
            except ValueError:
                pass

    def publish_range(self, publisher, distance, frame_id):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.52 # ~30 graus
        msg.min_range = 0.02     # 2 cm
        msg.max_range = 3.0      # 3 metros (definimos isso no Arduino)
        msg.range = distance
        publisher.publish(msg)

    def parse_odometry(self, line):
        # (Código antigo de odometria mantém-se igual, 
        # mas vou resumir aqui para garantir que encaixe)
        parts = line.split(',')
        if len(parts) < 6: return
        
        # Pega o theta direto do Arduino (IMU já integrado)
        # O,ticksL,ticksR,velL,velR,THETA
        current_theta = float(parts[5])
        
        # Cálculo simples de posição (X, Y) baseado na velocidade linear média
        # Se quiser usar o EKF depois, publicamos apenas a Odom crua.
        # Por enquanto, mantemos o cálculo simples para visualização:
        v_l = float(parts[3])
        v_r = float(parts[4])
        v_avg = (v_l + v_r) / 2.0
        
        dt = 0.05 # 50ms (intervalo do arduino)
        
        self.th = current_theta # Confia no IMU do Arduino
        self.x += v_avg * math.cos(self.th) * dt
        self.y += v_avg * math.sin(self.th) * dt

        # Publica Odom e TF (igual ao seu código anterior)
        # ... (Se precisar eu colo o bloco completo de Odom, 
        # mas assumo que você pode copiar do seu arquivo antigo se preferir)
        self.publish_odom_msg(v_avg, 0.0) # Simplificado

    def publish_odom_msg(self, v_lin, v_ang):
        # Cria quaternion
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Odom
        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v_lin
        o.twist.twist.angular.z = v_ang
        self.odom_pub.publish(o)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
