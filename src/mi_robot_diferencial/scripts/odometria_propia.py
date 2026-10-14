#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class Odometria(Node):
    def __init__(self):
        super().__init__('odometria_propia')

        # Parametros fisicos del robot
        self.r = 0.05
        self.L = 0.25

        # Pose inicial: posicion y orientacion
        self.x = self.y = self.theta = 0.0
        self.t0 = None

        # Publicar odometria y recibir velocidades de las ruedas
        self.pub = self.create_publisher(Odometry, '/odom_propia', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.calcular, 10)

    def calcular(self, msg):
        # Leer velocidades angulares segun el nombre de cada rueda
        try:
            der = msg.velocity[msg.name.index('joint_rueda_der')]
            izq = msg.velocity[msg.name.index('joint_rueda_izq')]
        except (ValueError, IndexError):
            return

        # 1. Cinematica directa: velocidades del robot
        v = self.r * (der + izq) / 2
        w = self.r * (der - izq) / self.L

        # Calcular el tiempo entre mensajes
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
            return
        dt = t - self.t0
        self.t0 = t

        # 2. Rotacion al marco fijo: velocidades globales
        vx = v * math.cos(self.theta)
        vy = v * math.sin(self.theta)

        # 3. Euler: integrar velocidades y actualizar la pose
        self.x += vx * dt
        self.y += vy * dt
        self.theta += w * dt

        # 4. Crear mensaje de odometria
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Posicion global
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Orientacion: convertir theta a cuaternion
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Velocidades locales del robot
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.pub.publish(odom)


rclpy.init()
nodo = Odometria()
rclpy.spin(nodo)
nodo.destroy_node()
rclpy.shutdown()