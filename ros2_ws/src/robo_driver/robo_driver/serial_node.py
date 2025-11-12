#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 0.2

def safe_float(x, default=0.0):
    try:
        v = float(x)
        if math.isnan(v) or math.isinf(v):
            return default
        return v
    except Exception:
        return default

class RoboDriverNode(Node):
    def __init__(self):
        super().__init__('serial_driver_node')

        # Params físicos
        self.TICKS_PER_REVOLUTION = 6550.0
        self.WHEEL_DIAMETER_M = 0.127
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER_M
        self.METERS_PER_TICK = self.WHEEL_CIRCUMFERENCE / max(self.TICKS_PER_REVOLUTION, 1.0)
        self.BASE_WIDTH = 0.66

        # Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
            self.get_logger().info(f"Porta Serial {SERIAL_PORT} aberta.")
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f"Erro ao abrir a porta serial {SERIAL_PORT}: {e}")
            raise SystemExit

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_callback, 10)

        # Timer 20 Hz
        self.timer = self.create_timer(1.0 / 20.0, self.serial_read_loop)

        # Estado
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_imu_rad = 0.0

        self.last_ticks_l = None
        self.last_ticks_r = None

        self.current_time = self.get_clock().now()
        self.last_time = self.current_time

        self.linear_vel_ms = 0.0
        self.angular_vel_rs = 0.0

    def cmd_vel_callback(self, twist_msg):
        linear_vel = safe_float(twist_msg.linear.x, 0.0)
        angular_vel = safe_float(twist_msg.angular.z, 0.0)

        # 🔹 Log para debug (mostra no terminal)
        print(f"[CMD_VEL] linear_x={linear_vel:.3f}, angular_z={angular_vel:.3f}", flush=True)
        # 🔹 Log pelo sistema de logger do ROS (opcional)
        self.get_logger().info(f"[CMD_VEL] linear_x={linear_vel:.3f}, angular_z={angular_vel:.3f}")

        cmd_string = f"V,{linear_vel:.2f},{angular_vel:.2f}\n"
        try:
            self.ser.write(cmd_string.encode('ascii'))
        except serial.SerialTimeoutException:
            self.get_logger().warn("Timeout ao enviar dados ao Arduino.")
    
    def serial_read_loop(self):
        try:
            processed = 0
            while self.ser.in_waiting > 0:
                data_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not data_line:
                    continue
                self.update_odometry(data_line)
                processed += 1
        except Exception as e:
            self.get_logger().error(f"Erro fatal no loop serial: {e}")

    def update_odometry(self, data_line: str):
        line = data_line.strip()

        # Sincronizar com 'O,' mais à direita
        if not line.startswith('O,'):
            idx = line.rfind('O,')
            if idx != -1:
                line = line[idx:]
            else:
                return

        # Parse: ["O", ticksL, ticksR, velL, velR, theta]
        parts_raw = line.split(',')
        parts = []
        for p in parts_raw:
            if len(parts) == 0:
                if p.strip() == 'O':
                    parts.append('O')
                else:
                    return
            else:
                parts.append(p.strip())
                if len(parts) == 6:
                    break

        if len(parts) != 6:
            self.get_logger().warn(f"Dados malformados (6 esperados, got {len(parts)}): '{line}'")
            return

        # Parse seguro
        current_ticks_l = safe_float(parts[1], None)
        current_ticks_r = safe_float(parts[2], None)
        vel_ticks_s_l = safe_float(parts[3], None)
        vel_ticks_s_r = safe_float(parts[4], None)
        theta_imu = safe_float(parts[5], None)

        if None in (current_ticks_l, current_ticks_r, vel_ticks_s_l, vel_ticks_s_r, theta_imu):
            self.get_logger().warn(f"Tokens inválidos: '{line}'")
            return

        # Wrap yaw para [-pi, pi]
        theta_imu = math.atan2(math.sin(theta_imu), math.cos(theta_imu))
        self.theta_imu_rad = theta_imu

        # Tempo
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        dt = max(1e-3, min(dt, 0.2))
        self.last_time = self.current_time

        # Inicialização
        if self.last_ticks_l is None or self.last_ticks_r is None:
            self.last_ticks_l = current_ticks_l
            self.last_ticks_r = current_ticks_r
            return

        # Deltas
        delta_ticks_l = current_ticks_l - self.last_ticks_l
        delta_ticks_r = current_ticks_r - self.last_ticks_r

        # Proteção contra saltos
        if abs(delta_ticks_l) > 1e6 or abs(delta_ticks_r) > 1e6:
            self.get_logger().warn("Ticks jump detected – re-sincronizando contadores")
            self.last_ticks_l = current_ticks_l
            self.last_ticks_r = current_ticks_r
            return  # ✅ AGORA O RETURN ESTÁ DENTRO DO IF

        # Distâncias (metros)
        dist_l = delta_ticks_l * self.METERS_PER_TICK
        dist_r = delta_ticks_r * self.METERS_PER_TICK

        # Velocidades por roda (m/s)
        vel_l_ms = vel_ticks_s_l * self.METERS_PER_TICK
        vel_r_ms = vel_ticks_s_r * self.METERS_PER_TICK

        # Velocidades do robô
        vx = (vel_l_ms + vel_r_ms) / 2.0
        wz = (vel_r_ms - vel_l_ms) / max(self.BASE_WIDTH, 1e-6)
        if math.isnan(vx) or math.isinf(vx): vx = 0.0
        if math.isnan(wz) or math.isinf(wz): wz = 0.0

        # Integra a pose usando yaw do IMU
        dist_center = (dist_l + dist_r) / 2.0
        max_step = 0.5
        if abs(dist_center) > max_step:
            self.get_logger().warn(f"Outlier dist_center={dist_center:.3f} m – descartando")
            dist_center = 0.0
        if math.isnan(dist_center) or math.isinf(dist_center):
            dist_center = 0.0

        self.x_pos += dist_center * math.cos(self.theta_imu_rad)
        self.y_pos += dist_center * math.sin(self.theta_imu_rad)

        # Normalização
        if math.isnan(self.x_pos) or math.isinf(self.x_pos): self.x_pos = 0.0
        if math.isnan(self.y_pos) or math.isinf(self.y_pos): self.y_pos = 0.0

        # Quaternion
        qz = math.sin(self.theta_imu_rad / 2.0)
        qw = math.cos(self.theta_imu_rad / 2.0)

        # Publica Odometry
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x_pos)
        odom.pose.pose.position.y = float(self.y_pos)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        odom.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)
    
        # ✅ Zerar twist quando parado
        if delta_ticks_l == 0 and delta_ticks_r == 0:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0

        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02
        ]

        self.odom_pub.publish(odom)
        
        # --- TF broadcaster: odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = self.current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.x_pos)
        t.transform.translation.y = float(self.y_pos)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

        self.last_ticks_l = current_ticks_l
        self.last_ticks_r = current_ticks_r

def main(args=None):
    rclpy.init(args=args)
    node = RoboDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
