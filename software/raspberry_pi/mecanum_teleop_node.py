#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from mecanum_motor_test import mecanum


CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ

# Gains PI conservés
KP = 0.35
KI = 0.45

PWM_MIN = -30
PWM_MAX = 30

I_MIN = -8
I_MAX = 8


# Rampe de variation PWM : limite les à-coups et les pics transitoires.
DUTY_RAMP_STEP = 2.0
STOP_RPM_EPS = 2.0

# Correction spécifique rotation
ROTATION_WZ_EPS = 0.02
TRANSLATION_V_EPS_MM_S = 1.0

# PWM minimal uniquement pour rotation pure r/f
# Si la rotation reste faible, tester 20.
# Si elle devient trop brutale, tester 14 ou 15.
PWM_MIN_ROT = 18


class MecanumTeleopNode(Node):
    def __init__(self):
        super().__init__("mecanum_teleop_node")

        self.chassis = mecanum.MecanumChassis(enable_debug=True)

        self.rpm_meas = [0.0, 0.0, 0.0, 0.0]
        self.i_term = [0.0, 0.0, 0.0, 0.0]

        # IMPORTANT :
        # duty_ol_base est la commande open-loop calculée à la réception de /cmd_vel.
        # Elle ne doit pas être remplacée par duty_pi à chaque période.
        self.duty_ol_base = [0, 0, 0, 0]

        self.last_cmd = {
            "vx_mm_s": 0.0,
            "vy_mm_s": 0.0,
            "wz_rad_s": 0.0,
            "velocity_mm_s": 0.0,
            "direction_deg": 0.0,
        }

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, "/esp32/wheel_rpm", self.rpm_callback, 10)

        log_dir = Path.home() / "ros2_ws" / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = log_dir / f"step6_pi_4wheels_rotation_fixed_{stamp}.csv"

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "t",
            "vx_mm_s", "vy_mm_s", "wz_rad_s",
            "velocity_mm_s", "direction_deg",
            "is_rotation_only",
            "rpm_ref_M1", "rpm_ref_M2", "rpm_ref_M3", "rpm_ref_M4",
            "rpm_meas_M1", "rpm_meas_M2", "rpm_meas_M3", "rpm_meas_M4",
            "err_M1", "err_M2", "err_M3", "err_M4",
            "p_M1", "p_M2", "p_M3", "p_M4",
            "i_M1", "i_M2", "i_M3", "i_M4",
            "motor_sign_M1", "motor_sign_M2", "motor_sign_M3", "motor_sign_M4",
            "duty_ol_base_M1", "duty_ol_base_M2", "duty_ol_base_M3", "duty_ol_base_M4",
            "duty_pi_M1", "duty_pi_M2", "duty_pi_M3", "duty_pi_M4",
        ])
        self.csv_file.flush()

        self.control_timer = self.create_timer(DT, self.control_step)

        self.get_logger().info("✅ mecanum_teleop_node PI 4 roues actif - rotation corrigée")
        self.get_logger().info(f"✅ KP={KP}, KI={KI}, DT={DT:.3f} s")
        self.get_logger().info(f"✅ PWM_MIN_ROT={PWM_MIN_ROT}")
        self.get_logger().info("✅ Correction : duty_ol_base figé à la réception de /cmd_vel")
        self.get_logger().info(f"✅ CSV : {self.csv_path}")

    def rpm_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.rpm_meas = [
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
                float(msg.data[3]),
            ]

    def cmd_vel_callback(self, msg: Twist):
        vx_mm_s = float(msg.linear.x) * 1000.0
        vy_mm_s = float(msg.linear.y) * 1000.0
        wz_rad_s = float(msg.angular.z)

        velocity_mm_s = math.sqrt(vx_mm_s * vx_mm_s + vy_mm_s * vy_mm_s)

        if abs(vx_mm_s) > 1e-9 or abs(vy_mm_s) > 1e-9:
            direction_deg = math.degrees(math.atan2(vy_mm_s, vx_mm_s))
        else:
            direction_deg = 0.0

        if direction_deg < 0.0:
            direction_deg += 360.0

        self.last_cmd = {
            "vx_mm_s": vx_mm_s,
            "vy_mm_s": vy_mm_s,
            "wz_rad_s": wz_rad_s,
            "velocity_mm_s": velocity_mm_s,
            "direction_deg": direction_deg,
        }

        # Calcul cinématique + application open-loop initiale.
        # Cette ligne garde le comportement validé des mouvements.
        self.chassis.set_velocity(
            velocity=velocity_mm_s,
            direction=direction_deg,
            angular_rate=wz_rad_s,
            fake=False
        )

        # IMPORTANT :
        # On sauvegarde ici le duty open-loop de base.
        # Ensuite, la boucle PI doit repartir de cette base,
        # et non pas du duty corrigé précédent.
        self.duty_ol_base = list(self.chassis.get_last_duty())

        # Si arrêt demandé, reset intégrales.
        if velocity_mm_s < TRANSLATION_V_EPS_MM_S and abs(wz_rad_s) < ROTATION_WZ_EPS:
            self.i_term = [0.0, 0.0, 0.0, 0.0]
            self.duty_ol_base = [0, 0, 0, 0]

    @staticmethod
    def clamp(value, vmin, vmax):
        return max(vmin, min(vmax, value))

    @staticmethod
    def sign_from_value(value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def is_rotation_only_command(self):
        return (
            self.last_cmd["velocity_mm_s"] < TRANSLATION_V_EPS_MM_S
            and abs(self.last_cmd["wz_rad_s"]) > ROTATION_WZ_EPS
        )

    def apply_rotation_min_pwm(self, duty_value, duty_ol_value, rpm_ref_value, motor_sign_value):
        """
        Compensation minimale uniquement pour rotation pure.
        Objectif : éviter que le PI donne un PWM trop faible et que le robot fasse :
        rotation -> pause -> rotation -> pause.
        """
        if abs(rpm_ref_value) < STOP_RPM_EPS:
            return 0

        if abs(duty_value) >= PWM_MIN_ROT:
            return duty_value

        # Le signe physique le plus fiable est celui du duty open-loop.
        sign = self.sign_from_value(duty_ol_value)

        # Sécurité si duty_ol est nul.
        if sign == 0:
            sign = self.sign_from_value(motor_sign_value * rpm_ref_value)

        if sign == 0:
            return duty_value

        return sign * PWM_MIN_ROT

    def control_step(self):
        rpm_ref = list(self.chassis.get_last_rpm_ref())
        rpm_meas = list(self.rpm_meas)

        # Correction majeure :
        # on utilise la base open-loop sauvegardée dans cmd_vel_callback.
        # On ne reprend pas self.chassis.get_last_duty() ici,
        # car get_last_duty() contient le duty_pi précédent après apply_motor_duty().
        duty_ol = list(self.duty_ol_base)

        motor_sign = list(self.chassis.motor_sign)

        is_rotation_only = self.is_rotation_only_command()

        err = [rpm_ref[i] - rpm_meas[i] for i in range(4)]
        p_term = [KP * err[i] for i in range(4)]

        duty_pi = [0, 0, 0, 0]

        for i in range(4):
            if abs(rpm_ref[i]) < STOP_RPM_EPS:
                self.i_term[i] = 0.0
                duty_pi[i] = 0
                continue

            i_candidate = self.i_term[i] + KI * err[i] * DT
            i_candidate = self.clamp(i_candidate, I_MIN, I_MAX)

            # err, P et I sont exprimés dans le signe logique de la roue.
            # Le PWM moteur doit respecter le signe physique réel du moteur.
            correction_pwm = motor_sign[i] * (p_term[i] + i_candidate)

            # Base open-loop fixe + correction PI
            u_unsat = duty_ol[i] + correction_pwm
            u_sat = self.clamp(u_unsat, PWM_MIN, PWM_MAX)

            # Anti-windup simple : on intègre seulement si pas de saturation.
            if abs(u_unsat - u_sat) < 1e-9:
                self.i_term[i] = i_candidate

            duty_value = int(round(u_sat))

            # Correction spécifique seulement pour r/f
            if is_rotation_only:
                duty_value = self.apply_rotation_min_pwm(
                    duty_value=duty_value,
                    duty_ol_value=duty_ol[i],
                    rpm_ref_value=rpm_ref[i],
                    motor_sign_value=motor_sign[i],
                )

            duty_pi[i] = int(self.clamp(duty_value, PWM_MIN, PWM_MAX))

        self.chassis.apply_motor_duty(
            duty1=duty_pi[0],
            duty2=duty_pi[1],
            duty3=duty_pi[2],
            duty4=duty_pi[3],
        )

        self.csv_writer.writerow([
            time.time(),
            self.last_cmd["vx_mm_s"],
            self.last_cmd["vy_mm_s"],
            self.last_cmd["wz_rad_s"],
            self.last_cmd["velocity_mm_s"],
            self.last_cmd["direction_deg"],
            int(is_rotation_only),
            rpm_ref[0], rpm_ref[1], rpm_ref[2], rpm_ref[3],
            rpm_meas[0], rpm_meas[1], rpm_meas[2], rpm_meas[3],
            err[0], err[1], err[2], err[3],
            p_term[0], p_term[1], p_term[2], p_term[3],
            self.i_term[0], self.i_term[1], self.i_term[2], self.i_term[3],
            motor_sign[0], motor_sign[1], motor_sign[2], motor_sign[3],
            duty_ol[0], duty_ol[1], duty_ol[2], duty_ol[3],
            duty_pi[0], duty_pi[1], duty_pi[2], duty_pi[3],
        ])
        self.csv_file.flush()

        mode = "ROT" if is_rotation_only else "MOVE"

        self.get_logger().info(
            f"PI4_{mode} | "
            f"M1 ref/meas={rpm_ref[0]:7.2f}/{rpm_meas[0]:7.2f} duty={duty_pi[0]:4d} | "
            f"M2 ref/meas={rpm_ref[1]:7.2f}/{rpm_meas[1]:7.2f} duty={duty_pi[1]:4d} | "
            f"M3 ref/meas={rpm_ref[2]:7.2f}/{rpm_meas[2]:7.2f} duty={duty_pi[2]:4d} | "
            f"M4 ref/meas={rpm_ref[3]:7.2f}/{rpm_meas[3]:7.2f} duty={duty_pi[3]:4d}"
        )

    def destroy_node(self):
        try:
            self.chassis.reset_motors()
        except Exception:
            pass

        try:
            self.csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
