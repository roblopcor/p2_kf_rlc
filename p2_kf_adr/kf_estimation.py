import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from .visualization import Visualizer

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator, generate_noisy_measurement
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        self.declare_parameter('noise_config', 'low')  # valor por defecto = 'low'
        noise_config = self.get_parameter('noise_config').get_parameter_value().string_value # Cambiar entre 'low', 'high_measurement', 'high_process'

        # Nodo visualizador
        self.visualizer = Visualizer()

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1
        self.last_time = None         

        if noise_config == 'low':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.02, 0.02, 0.01]
            self.obs_noise_std = [0.02, 0.02, 0.01]
        elif noise_config == 'high_measurement':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.02, 0.02, 0.01]
            self.obs_noise_std = [0.1, 0.1, 0.05]  # Aumento del ruido de medición
        elif noise_config == 'high_process':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.1, 0.1, 0.05]  # Aumento del ruido de proceso
            self.obs_noise_std = [0.02, 0.02, 0.01]
        else:
            self.get_logger().warn(f"Valor de noise_config desconocido: {noise_config}, usando configuración por defecto.")
            proc_noise_std = [0.02, 0.02, 0.01]
            self.obs_noise_std = [0.02, 0.02, 0.01]

        self.kf = KalmanFilter(initial_state, initial_covariance, proc_noise_std, self.obs_noise_std)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # Paso 1: Extraer pose y velocidades de la odometría
        pose = odom_to_pose2D(msg)  # (x, y, theta)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        u = np.array([v, omega])

        # Paso 2: Calcular delta_t
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Paso 3: Ejecutar predicción y corrección del filtro
        self.kf.predict(u, dt)

        # Observación ruidosa (mismo ruido con el que se diseña nuestro filtro)
        z_full = generate_noisy_measurement(pose, v, omega, self.obs_noise_std + [0.02, 0.02])
        z = z_full[:3]  # extraer solo x, y, theta con ruido
        self.kf.update(z)
 
        # Paso 4: Publicar estado estimado
        self.publish_estimate(msg.header.stamp)

        # Visualizador en Rviz y MatplotLib:
        real_pose = pose  # (x, y, theta) directamente de la odometría
        estimated_pose = self.kf.get_state()
        covariance = self.kf.get_covariance()

        self.visualizer.update(real_pose, estimated_pose, covariance)

    def publish_estimate(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"

        x, y, theta = self.kf.get_state()
        covariance = self.kf.get_covariance()

        # Rellenar posición
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convertir theta a cuaternión
        q = self.yaw_to_quaternion(theta)
        msg.pose.pose.orientation = q

        # Rellenar covarianza (6x6): solo usamos los primeros 3 elementos
        # x, y, y yaw -> posiciones 0, 7 y 35
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = covariance[0, 0]
        msg.pose.covariance[1] = covariance[0, 1]
        msg.pose.covariance[5] = covariance[0, 2]
        msg.pose.covariance[6] = covariance[1, 0]
        msg.pose.covariance[7] = covariance[1, 1]
        msg.pose.covariance[11] = covariance[1, 2]
        msg.pose.covariance[30] = covariance[2, 0]
        msg.pose.covariance[31] = covariance[2, 1]
        msg.pose.covariance[35] = covariance[2, 2]

        self.publisher.publish(msg)

    def yaw_to_quaternion(self, yaw):
        """
        Convierte un ángulo en radianes a un quaternion tipo ROS
        """
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
 