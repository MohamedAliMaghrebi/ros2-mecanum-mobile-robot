#!/usr/bin/env python3
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_mecanum')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            "Téléop clavier active. Appuyez sur 'x' ou Ctrl+C pour quitter."
        )

        # Vitesse linéaire (m/s) et vitesse angulaire (rad/s)
        self.speed = 0.05
        self.turn_speed = 0.5

        # Mapping touches -> (vx, vy, wz)
        self.key_mapping = {
            'w': ( self.speed,  0.0,        0.0),          # Avant
            's': (-self.speed,  0.0,        0.0),          # Arrière
            'a': ( 0.0,         self.speed, 0.0),          # Gauche
            'd': ( 0.0,        -self.speed, 0.0),          # Droite
            'q': ( self.speed,  self.speed, 0.0),          # Avant-Gauche
            'e': ( self.speed, -self.speed, 0.0),          # Avant-Droite
            'z': (-self.speed,  self.speed, 0.0),          # Arrière-Gauche
            'c': (-self.speed, -self.speed, 0.0),          # Arrière-Droite
            'r': ( 0.0,         0.0,        self.turn_speed),   # Rotation horaire
            'f': ( 0.0,         0.0,       -self.turn_speed),   # Rotation antihoraire
        }

        self.settings = None  # sera initialisé dans run()

    def get_key(self) -> str:
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_stop(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def run(self) -> None:
        self.settings = termios.tcgetattr(sys.stdin)

        twist = Twist()

        try:
            while rclpy.ok():
                key = self.get_key()

                # Quitter : Ctrl+C ou 'x'
                if key == '\x03' or key == 'x':
                    break

                if key in self.key_mapping:
                    vx, vy, wz = self.key_mapping[key]
                    twist.linear.x = float(vx)
                    twist.linear.y = float(vy)
                    twist.angular.z = float(wz)

                    self.publisher.publish(twist)
                    self.get_logger().info(
                        f"vx={vx:.2f} m/s, vy={vy:.2f} m/s, ω={wz:.2f} rad/s"
                    )
                else:
                    # Stop par défaut si touche non reconnue
                    self.publish_stop()

        finally:
            # Stop final + restauration du terminal
            self.publish_stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info("Téléop arrêtée.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
