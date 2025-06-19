import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState 
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from motor_board_md49_ros2.motor_board import MotorBoardMD49

class MotorBoardNode(Node):
    def __init__(self):
        super().__init__('motor_board_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baudrate', 38400),
                ('max_linear_speed', 1.0),  # Velocidad lineal máxima en m/s
                ('max_angular_speed', 1.0), # Velocidad angular máxima en rad/s
                ('wheel_separation', 0.5),  # Distancia entre ruedas (metros)
                ('wheel_radius', 0.1),      # Radio de ruedas (metros)
                ('publish_odometry', False)  # Publicar odometría
            ]
        )

        # Parámetros físicos del robot
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        # Parámetros de las ruedas
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.pub_odometry = self.get_parameter('publish_odometry').value

        self.initialize_motor_board()

        # Suscribirse al tópico /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publicar odometría
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        # TransformBroadcaster para odometría
        self.tf_broadcaster = TransformBroadcaster(self)
        # Publicar estados de las articulaciones
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)


        # Estado de odometría
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Posiciones de las ruedas
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Publicar la velocidad de los motores
        self.speed1_publisher = self.create_publisher(Int32, '/motor_speed_1', 10)
        self.speed2_publisher = self.create_publisher(Int32, '/motor_speed_2', 10)

        # Timer para leer datos periódicamente
        self.timer = self.create_timer(0.1, self.publish_motor_speeds)
        self.get_logger().info("MotorBoardNode inicializado")

    
    def initialize_motor_board(self):
        """Inicializa la conexión con la placa MD49."""
        try:
            self.motor_board = MotorBoardMD49(self.serial_port, self.baudrate)
            self.get_logger().info("Intentando conectar con MotorBoardMD49...")
            if self.check_connection():
                self.get_logger().info("Conexión MD49 exitosa!")
            else:
                raise RuntimeError("Falló la conexión con MD49")
        except Exception as e:
            self.get_logger().error(f"Error al inicializar MotorBoardMD49: {e}")
            raise

    def check_connection(self):
        """
        Método para verificar la conexión con la placa.
        Envía un comando básico y verifica si hay respuesta.
        """
        try:
            # Intentar leer el voltaje de la batería como prueba
            voltage = self.motor_board.GetVolts()
            if voltage is not None:
                self.get_logger().info(f"Voltaje de batería detectado: {voltage}V")
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Error al verificar la conexión: {e}")
            return False


    def cmd_vel_callback(self, msg: Twist):
        """Convierte Twist a velocidades del MD49."""
        # Limitar velocidades
        linear = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

        # Calcular velocidades de ruedas (differential drive)
        left = (linear - angular * self.wheel_sep / 2) / self.wheel_rad
        right = (linear + angular * self.wheel_sep / 2) / self.wheel_rad

        # Mapear a rango MD49 (0-255)
        left_mapped = self.map_speed(left)
        right_mapped = self.map_speed(right)

        # Enviar comandos a los motores
        self.get_logger().debug(f"Motores: L={left_mapped} | R={right_mapped}")

        self.motor_board.SetSpeed1(left_mapped)
        self.motor_board.set_speed_2_turn(right_mapped)
        
    def map_speed(self, speed):
        """Convierte velocidad en m/s a rango MD49 (0-255)"""
        # MAX_MOTOR_SPEED = 255
        NEUTRAL = 128
        
        # Escalar velocidad (-1 a 1) -> (-127 a 127)
        mapped = int(speed * 127)
        # Aplicar offset para rango MD49
        return max(0, min(NEUTRAL + mapped, 255))

    def map_velocity(self, velocity):
        """Mapea una velocidad [-1, 1] al rango 0-255 del MD49."""
        # Escalado no lineal para precisión alrededor del stop (128)
        if velocity >= 0:
            return 128 + int(velocity * 127)  # 128-255: forward
        else:
            return 128 + int(velocity * 128)  # 0-128: reverse

    def publish_motor_speeds(self):
        """Publica las velocidades actuales."""
        speed1 = self.motor_board.GetSpeed1()
        speed2 = self.motor_board.GetSpeed2()

        self.get_logger().info(f"publish_motor_speeds velocidad a motores 1:  {speed1}  --- 2:  {speed2} ")

        if speed1 is not None:
            self.speed1_publisher.publish(Int32(data=speed1))
        if speed2 is not None:
            self.speed2_publisher.publish(Int32(data=speed2))

        # Publicar estados de las articulaciones
        self.publish_joint_states(speed1, speed2)
        
        # Publicar odometría
        if self.pub_odometry:
            # Solo publicar odometría si está habilitado
            self.get_logger().info("Publicando odometría...")
            self.publish_odometry(speed1, speed2)

    def publish_joint_states(self, speed1, speed2):
        """Publica el estado de las articulaciones."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # Convertir velocidades a rad/s normalizados
        left_vel = (speed1 - 128) / 127.0
        right_vel = (speed2 - 128) / 127.0

        v_left = left_vel * self.wheel_rad
        v_right = right_vel * self.wheel_rad

        # Actualizar posición angular
        self.left_wheel_pos += v_left * dt / self.wheel_rad
        self.right_wheel_pos += v_right * dt / self.wheel_rad

        # Publicar joint_states
        joint_state = JointState()
        joint_state.header.frame_id = "base_footprint"
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_state_publisher.publish(joint_state)

    def publish_odometry(self, speed1, speed2):
        # Tiempo actual y delta t
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Velocidades en rad/s
        left_vel = (speed1 - 128) / 127.0  # Normalizado [-1, 1]
        right_vel = (speed2 - 128) / 127.0

        # Convertir a m/s
        v_left = left_vel * self.wheel_rad
        v_right = right_vel * self.wheel_rad

        # Cinemática diferencial
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.wheel_sep

        delta_x = v * math.cos(self.th) * dt
        delta_y = v * math.sin(self.th) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Construir mensaje Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        #odom.child_frame_id = "base_link"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        print("odometria ->", odom.pose.pose.position)

        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_publisher.publish(odom)

        # Publicar la transformación tf de odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.header.frame_id = "base_footprint"
        #t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)


    def destroy_node(self):
        self.motor_board.Close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MotorBoardNode()
    node.get_logger().info("Iniciando nodo")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando el nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()