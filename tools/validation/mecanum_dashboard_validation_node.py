#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ============================================================
# MECANUM DASHBOARD VALIDATION NODE
# ============================================================
#
# Objectif :
# - garder mecanum_step_trajectory_node.py pour le rectangle ;
# - créer un fichier séparé pour valider le dashboard ;
# - tester un seul mouvement à la fois ;
# - comparer réalité vs dashboard.
#
# Ce noeud publie seulement /cmd_vel.
# Il ne calcule pas l'odométrie.
# Le dashboard reste seulement observateur.
#
# Convention validée :
# X+  = avancer
# X-  = reculer
# Y+  = gauche
# Y-  = droite
# WZ+ = rotation horaire
# WZ- = rotation antihoraire
#
# ============================================================


# ============================================================
# 1. FRÉQUENCE DE PUBLICATION
# ============================================================

PUBLISH_HZ = 20.0
DT = 1.0 / PUBLISH_HZ


# ============================================================
# 2. PARAMÈTRES VALIDÉS EXPÉRIMENTALEMENT
# ============================================================

CMD_LINEAR = 0.035
CMD_ANGULAR = 0.12

X_TIME = 2.50      # environ 30 cm en X
Y_TIME = 16.00      # environ 20 cm en Y
WZ_TIME = 3.55     # rotation proche de 90 degrés

STOP_TIME = 2.00


# ============================================================
# 3. CHOISIR LE TEST ICI
# ============================================================
#
# Tests possibles :
#
# "x_plus"     : avancer environ 30 cm
# "x_minus"    : reculer environ 30 cm
# "y_plus"     : gauche environ 20 cm
# "y_minus"    : droite environ 20 cm
# "wz_plus"    : rotation horaire proche de 90°
# "wz_minus"   : rotation antihoraire proche de 90°
#
# Pour valider le dashboard, il faut tester un seul mode à la fois.
#

TEST_MODE = "x_plus"


class MecanumDashboardValidationNode(Node):
    def __init__(self):
        super().__init__("mecanum_dashboard_validation_node")

        # Publisher vers /cmd_vel.
        # C'est le même topic que le clavier et le rectangle.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("================================================")
        self.get_logger().info("MECANUM DASHBOARD VALIDATION NODE")
        self.get_logger().info("Test unitaire pour comparer réalité / dashboard")
        self.get_logger().info("================================================")
        self.get_logger().info(f"PUBLISH_HZ  = {PUBLISH_HZ:.1f} Hz")
        self.get_logger().info(f"DT          = {DT:.3f} s")
        self.get_logger().info(f"CMD_LINEAR  = {CMD_LINEAR:.3f} m/s")
        self.get_logger().info(f"CMD_ANGULAR = {CMD_ANGULAR:.3f} rad/s")
        self.get_logger().info(f"X_TIME      = {X_TIME:.2f} s")
        self.get_logger().info(f"Y_TIME      = {Y_TIME:.2f} s")
        self.get_logger().info(f"WZ_TIME     = {WZ_TIME:.2f} s")
        self.get_logger().info(f"STOP_TIME   = {STOP_TIME:.2f} s")
        self.get_logger().info(f"TEST_MODE   = {TEST_MODE}")
        self.get_logger().info("================================================")

    def publish_cmd_once(self, vx, vy, wz):
        """
        Publier une seule commande Twist sur /cmd_vel.

        vx : vitesse suivant X en m/s
        vy : vitesse suivant Y en m/s
        wz : vitesse angulaire suivant Z en rad/s
        """

        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)

        self.cmd_pub.publish(msg)

    def stop_robot(self, duration=STOP_TIME):
        """
        Envoyer vx=0, vy=0, wz=0 pendant une durée donnée.
        """

        self.get_logger().info(
            f"ARRÊT : vx=0.000 | vy=0.000 | wz=0.000 pendant {duration:.2f} s"
        )

        start = time.perf_counter()

        while rclpy.ok():
            elapsed = time.perf_counter() - start

            if elapsed >= duration:
                break

            self.publish_cmd_once(0.0, 0.0, 0.0)
            time.sleep(DT)

    def hold_command(self, vx, vy, wz, duration, label):
        """
        Maintenir une commande constante pendant une durée donnée.
        """

        self.get_logger().info("------------------------------------------------")
        self.get_logger().info(label)
        self.get_logger().info(
            f"Commande envoyée : vx={vx:.3f} m/s | vy={vy:.3f} m/s | wz={wz:.3f} rad/s"
        )
        self.get_logger().info(f"Durée commandée  : {duration:.2f} s")

        start = time.perf_counter()

        while rclpy.ok():
            elapsed = time.perf_counter() - start

            if elapsed >= duration:
                break

            self.publish_cmd_once(vx, vy, wz)
            time.sleep(DT)

        self.stop_robot(STOP_TIME)

    # ========================================================
    # TESTS UNITAIRES
    # ========================================================

    def test_x_plus(self):
        self.hold_command(
            vx=+CMD_LINEAR,
            vy=0.0,
            wz=0.0,
            duration=X_TIME,
            label="TEST DASHBOARD X+ : avancer environ 30 cm"
        )

    def test_x_minus(self):
        self.hold_command(
            vx=-CMD_LINEAR,
            vy=0.0,
            wz=0.0,
            duration=X_TIME,
            label="TEST DASHBOARD X- : reculer environ 30 cm"
        )

    def test_y_plus(self):
        self.hold_command(
            vx=0.0,
            vy=+CMD_LINEAR,
            wz=0.0,
            duration=Y_TIME,
            label="TEST DASHBOARD Y+ : gauche environ 20 cm"
        )

    def test_y_minus(self):
        self.hold_command(
            vx=0.0,
            vy=-CMD_LINEAR,
            wz=0.0,
            duration=Y_TIME,
            label="TEST DASHBOARD Y- : droite environ 20 cm"
        )

    def test_wz_plus(self):
        self.hold_command(
            vx=0.0,
            vy=0.0,
            wz=+CMD_ANGULAR,
            duration=WZ_TIME,
            label="TEST DASHBOARD WZ+ : rotation horaire proche de 90 deg"
        )

    def test_wz_minus(self):
        self.hold_command(
            vx=0.0,
            vy=0.0,
            wz=-CMD_ANGULAR,
            duration=WZ_TIME,
            label="TEST DASHBOARD WZ- : rotation antihoraire proche de 90 deg"
        )

    def run_test(self):
        """
        Exécuter le test choisi par TEST_MODE.
        """

        self.get_logger().info("Attente initiale 2 s pour connexion ROS 2...")
        time.sleep(2.0)

        # Sécurité avant mouvement
        self.stop_robot(2.0)

        if TEST_MODE == "x_plus":
            self.test_x_plus()

        elif TEST_MODE == "x_minus":
            self.test_x_minus()

        elif TEST_MODE == "y_plus":
            self.test_y_plus()

        elif TEST_MODE == "y_minus":
            self.test_y_minus()

        elif TEST_MODE == "wz_plus":
            self.test_wz_plus()

        elif TEST_MODE == "wz_minus":
            self.test_wz_minus()

        else:
            self.get_logger().error(f"TEST_MODE inconnu : {TEST_MODE}")

        # Sécurité après mouvement
        self.stop_robot(2.0)

        self.get_logger().info("================================================")
        self.get_logger().info("TEST DASHBOARD TERMINÉ")
        self.get_logger().info("Maintenant, noter les valeurs affichées :")
        self.get_logger().info("x final, y final, theta final")
        self.get_logger().info("Puis comparer avec la mesure réelle.")
        self.get_logger().info("================================================")


def main(args=None):
    rclpy.init(args=args)

    node = MecanumDashboardValidationNode()

    try:
        node.run_test()
    except KeyboardInterrupt:
        node.get_logger().info("Interruption clavier. Arrêt du robot.")
        node.stop_robot(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
