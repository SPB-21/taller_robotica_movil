#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalizar(a):
    return math.atan2(math.sin(a), math.cos(a))


class Comparador(Node):
    def __init__(self):
        super().__init__('comparar_odometrias')

        self.declare_parameter('csv_path', '/tmp/error_odometria.csv')
        ruta = self.get_parameter('csv_path').value

        # Ultima pose recibida del plugin DiffDrive
        self.ref = None
        self.n = 0
        self.suma_pos2 = 0.0
        self.suma_yaw2 = 0.0
        self.ultimo = (0.0, 0.0)

        self.archivo = open(ruta, 'w', newline='')
        self.csv = csv.writer(self.archivo)
        self.csv.writerow([
            't', 'x_plugin', 'y_plugin', 'yaw_plugin',
            'x_propia', 'y_propia', 'yaw_propia',
            'error_x', 'error_y', 'error_pos', 'error_yaw'])

        self.create_subscription(Odometry, '/odom', self.cb_plugin, 10)
        self.create_subscription(Odometry, '/odom_propia', self.cb_propia, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/error_odometria', 10)
        self.create_timer(2.0, self.reportar)
        self.get_logger().info(f'Comparando /odom vs /odom_propia. CSV: {ruta}')

    def cb_plugin(self, msg):
        p = msg.pose.pose
        self.ref = (p.position.x, p.position.y, yaw_de(p.orientation))

    def cb_propia(self, msg):
        if self.ref is None:
            return
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_de(p.orientation)
        ex = x - self.ref[0]
        ey = y - self.ref[1]
        epos = math.hypot(ex, ey)
        eyaw = normalizar(yaw - self.ref[2])

        self.n += 1
        self.suma_pos2 += epos ** 2
        self.suma_yaw2 += eyaw ** 2
        self.ultimo = (epos, eyaw)

        out = Float64MultiArray()
        out.data = [ex, ey, epos, eyaw]
        self.pub.publish(out)

        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv.writerow([t, *self.ref, x, y, yaw, ex, ey, epos, eyaw])

    def reportar(self):
        if self.n == 0:
            return
        rmse_pos = math.sqrt(self.suma_pos2 / self.n)
        rmse_yaw = math.sqrt(self.suma_yaw2 / self.n)
        self.get_logger().info(
            f'error pos = {self.ultimo[0]:.4f} m | error yaw = '
            f'{math.degrees(self.ultimo[1]):.2f} deg | '
            f'RMSE pos = {rmse_pos:.4f} m | RMSE yaw = {math.degrees(rmse_yaw):.2f} deg')
        self.archivo.flush()


def main():
    rclpy.init()
    nodo = Comparador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.archivo.close()
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
