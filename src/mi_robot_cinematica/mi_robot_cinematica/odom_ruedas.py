#!/usr/bin/env python3
"""
odom_ruedas.py — Odometría propia a partir de las velocidades de las ruedas.

    ros2 run mi_robot_cinematica odom_ruedas

Cinemática (w -> u -> q̇):

    w = [wD, wI]ᵀ                     (rad/s, de /joint_states)

    u = [v, ω]ᵀ = M w,   M = [ r/2   r/2 ]
                             [ r/L  -r/L ]

    q̇ = [ẋ, ẏ, θ̇]ᵀ = J(θ) u,   J(θ) = [cosθ  0]
                                       [sinθ  0]
                                       [ 0    1]

Se integra q y se compara con el /odom del plugin DiffDrive.
Publica:  /odom_ruedas (Odometry)  y  /error_odom [ex, ey, eθ, e_pos]
Guarda:   ~/error_odom.csv  (para las gráficas del informe)
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

# ===== Parámetros físicos (deben coincidir con el xacro) =====
R = 0.05    # rueda_radio [m]
L = 0.25    # separacion_ruedas [m]

# ===== Matriz M: de velocidades de rueda a velocidades del robot =====
M = np.array([[R / 2.0,  R / 2.0],
              [R / L,   -R / L]])


def envolver(a):
    """Ángulo en [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_quat(q):
    """Ángulo θ (yaw) a partir del cuaternión del mensaje Odometry."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomRuedas(Node):
    def __init__(self):
        super().__init__('odom_ruedas')

        self.q = np.zeros(3)        # pose integrada [x, y, θ], arranca en 0 igual que /odom
        self.t_ant = None           # tiempo del mensaje anterior
        self.err_max = 0.0
        self.suma_err2 = 0.0
        self.n = 0
        self.ultimo_error = [0.0, 0.0, 0.0, 0.0]

        self.create_subscription(JointState, '/joint_states', self.cb_ruedas, 50)
        self.create_subscription(Odometry, '/odom', self.cb_odom_plugin, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_ruedas', 10)
        self.pub_err = self.create_publisher(Float64MultiArray, '/error_odom', 10)
        self.create_timer(1.0, self.reportar)   # imprime el error cada segundo

        ruta = os.path.expanduser('~/error_odom.csv')
        self.csv = open(ruta, 'w')
        self.csv.write('t,x_plugin,y_plugin,th_plugin,x_ruedas,y_ruedas,th_ruedas,ex,ey,eth,e_pos\n')
        self.get_logger().info(f'Odometría por ruedas iniciada. CSV en {ruta}')

    # ---------- 1) Leer w de /joint_states e integrar q ----------
    def cb_ruedas(self, msg):
        if 'joint_rueda_der' not in msg.name or 'joint_rueda_izq' not in msg.name:
            return
        if len(msg.velocity) < len(msg.name):
            return

        iD = msg.name.index('joint_rueda_der')
        iI = msg.name.index('joint_rueda_izq')
        w = np.array([msg.velocity[iD], msg.velocity[iI]])   # w = [wD, wI]

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_ant is None:
            self.t_ant = t
            return
        dt = t - self.t_ant
        self.t_ant = t
        if dt <= 0.0 or dt > 0.5:     # mensaje repetido o salto de tiempo
            return

        # Paso 1: u = M w
        v, om = M @ w

        # Paso 2: q̇ = J(θ) u, evaluada en el ángulo medio del paso (más precisa que Euler)
        th_medio = self.q[2] + om * dt / 2.0
        J = np.array([[math.cos(th_medio), 0.0],
                      [math.sin(th_medio), 0.0],
                      [0.0,                1.0]])
        q_dot = J @ np.array([v, om])

        # Integración: q(k+1) = q(k) + q̇ dt
        self.q = self.q + q_dot * dt

        # Publicar la odometría propia (para verla en RViz)
        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_ruedas'
        od.pose.pose.position.x = float(self.q[0])
        od.pose.pose.position.y = float(self.q[1])
        od.pose.pose.orientation.z = math.sin(self.q[2] / 2.0)
        od.pose.pose.orientation.w = math.cos(self.q[2] / 2.0)
        od.twist.twist.linear.x = float(v)
        od.twist.twist.angular.z = float(om)
        self.pub_odom.publish(od)

    # ---------- 2) Comparar con el /odom del plugin DiffDrive ----------
    def cb_odom_plugin(self, msg):
        if self.t_ant is None:
            return
        xp = msg.pose.pose.position.x
        yp = msg.pose.pose.position.y
        thp = yaw_de_quat(msg.pose.pose.orientation)

        x, y, th = self.q
        ex = xp - x
        ey = yp - y
        eth = envolver(thp - th)
        e_pos = math.hypot(ex, ey)

        self.ultimo_error = [ex, ey, eth, e_pos]
        self.err_max = max(self.err_max, e_pos)
        self.suma_err2 += e_pos ** 2
        self.n += 1

        err = Float64MultiArray()
        err.data = [ex, ey, eth, e_pos]
        self.pub_err.publish(err)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.write(f'{t:.3f},{xp:.5f},{yp:.5f},{thp:.5f},{x:.5f},{y:.5f},'
                       f'{envolver(th):.5f},{ex:.5f},{ey:.5f},{eth:.5f},{e_pos:.5f}\n')

    # ---------- 3) Reporte en consola ----------
    def reportar(self):
        if self.n == 0:
            return
        ex, ey, eth, e_pos = self.ultimo_error
        rms = math.sqrt(self.suma_err2 / self.n)
        self.get_logger().info(
            f'Error pos: {e_pos * 1000:.1f} mm | eθ: {math.degrees(eth):.2f}° | '
            f'RMS: {rms * 1000:.1f} mm | máx: {self.err_max * 1000:.1f} mm')


def main():
    rclpy.init()
    nodo = OdomRuedas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.csv.close()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()