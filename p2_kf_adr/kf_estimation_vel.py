import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        self.declare_parameter("noise_config", "low")  # Parámetro configurable
        noise_config = self.get_parameter("noise_config").get_parameter_value().string_value

        # Configurar niveles de ruido
        if noise_config == 'low':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.02] * 6
            obs_noise_std = [0.02] * 6
        elif noise_config == 'high_measurement':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.02] * 6
            obs_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.05]
        elif noise_config == 'high_process':
            self.get_logger().info(f"Valor de ruido como: {noise_config}")
            proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.05]
            obs_noise_std = [0.02] * 6
        else:
            self.get_logger().warn(f"Valor de noise_config desconocido: {noise_config}, usando configuración por defecto.")
            proc_noise_std = [0.02] * 6
            obs_noise_std = [0.02] * 6

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.visualizer = Visualizer()

        self.last_time = None

        self.kf = KalmanFilter_2(initial_state, initial_covariance, proc_noise_std, obs_noise_std)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

    def odom_callback(self, msg):
        # Extraer datos
        pose = odom_to_pose2D(msg)
        x, y, theta = pose

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Convertir vx, vy desde velocidad lineal + theta
        vx = v * math.cos(theta)
        vy = v * math.sin(theta)

        z = np.array([x, y, theta, vx, vy, omega])

        # Calcular dt
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Filtro de Kalman
        self.kf.predict(dt=dt)
        self.kf.update(z)

        # Publicar resultado
        self.publish_estimate(msg.header.stamp)

        # Actualizar visualización
        est_state = self.kf.get_state()
        self.visualizer.update(
            real_pose=[x, y, theta],
            estimated_pose=est_state[:3],
            covariance=self.kf.get_covariance()[:3, :3]
        )

    def publish_estimate(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"

        x, y, theta = self.kf.get_state()[:3]
        covariance = self.kf.get_covariance()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convertir theta a cuaternión
        q = Quaternion()
        q.w = math.cos(theta / 2.0)
        q.z = math.sin(theta / 2.0)
        q.x = 0.0
        q.y = 0.0
        msg.pose.pose.orientation = q

        # Copiar las covarianzas x, y, theta
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

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

