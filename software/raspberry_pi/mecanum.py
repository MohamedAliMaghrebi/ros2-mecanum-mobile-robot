#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from mecanum_motor_test import ros_robot_controller_sdk as rrc


# Instance globale de la carte
board = rrc.Board()


class MecanumChassis:
    """
    Châssis Mecanum avec diagnostic complet.

    Convention robot :
    M1 = roue avant gauche
    M2 = roue avant droite
    M3 = roue arrière gauche
    M4 = roue arrière droite

    Entrées :
    - velocity     : mm/s
    - direction    : degrés
    - angular_rate : rad/s

    Affichages :
    - [CMD]       : commande reçue
    - [KIN]       : cinématique vx, vy, vp
    - [V_LOGIC]   : vitesse logique roue avant application des signes moteurs
    - [PWM_CALC]  : PWM calculé après signes matériels
    - [PWM_SEND]  : PWM envoyé réellement à la carte
    """

    def __init__(
        self,
        a: float = 175.0,
        b: float = 115.0,
        wheel_diameter: float = 65.0,
        enable_debug: bool = True
    ):
        # Géométrie du robot en mm
        self.a = float(a)
        self.b = float(b)

        # Roue en mm
        self.wheel_diameter = float(wheel_diameter)
        self.wheel_radius = self.wheel_diameter / 2.0

        # Debug console
        self.enable_debug = bool(enable_debug)

        # Limites de sécurité PWM
        self.pwm_limit = 35

        # Petite zone morte : si le PWM calculé est très petit, on force à 0
        # Très utile pour les diagonales où certaines roues doivent être arrêtées.
        self.duty_deadzone = 2

        # Modèle feedforward expérimental : RPM_ref -> PWM open-loop.
        # Objectif : sortir de la zone morte moteur et donner au PI
        # seulement une petite erreur résiduelle à corriger.
        #
        # Modèle utilisé :
        # PWM_abs = ff_pwm_offset + ff_pwm_slope * |rpm_ref|
        #
        # Valeurs initiales issues des tests PWM/RPM sur tapis :
        # autour de 10-12 RPM, le PWM utile est environ 16.
        self.ff_pwm_offset = 13.0
        self.ff_pwm_slope = 0.29
        self.ff_pwm_min_moving = 16.0
        self.ff_pwm_max_open_loop = 24.0
        self.rpm_deadband = 1.0

        # Feedforward séparé translation / rotation.
        #
        # Diagnostic stable actuel :
        # TRANSLATION_X : rpm_meas > rpm_ref, donc PWM trop fort.
        # ROTATION_WZ  : rpm_meas < rpm_ref, donc PWM trop faible.
        #
        # Objectif :
        # - translation autour de 12 PWM
        # - rotation autour de 21 PWM
        self.ff_trans_offset = 7.5
        self.ff_trans_slope = 0.35
        self.ff_trans_min_moving = 13.0
        self.ff_trans_max_open_loop = 18.0

        self.ff_rot_offset = 14.0
        self.ff_rot_slope = 0.30
        self.ff_rot_min_moving = 18.0
        self.ff_rot_max_open_loop = 22.0

        # Calibration expérimentale vitesse roue -> PWM.
        # Ancienne loi : duty = v_roue_mm_s.
        # Test sur tapis : v_roue = 20 mm/s donnait PWM = 20,
        # mais rpm_meas était beaucoup trop élevé.
        # Première correction : 20 mm/s -> PWM ≈ 15.
        # Nouvelle correction : 20 mm/s -> PWM ≈ 18, donc gain = 0.90.
        # Dernières valeurs mémorisées
        self.velocity = 0.0
        self.direction = 0.0
        self.angular_rate = 0.0

        self.v_mm_s = (0.0, 0.0, 0.0, 0.0)
        self.w_rad_s = (0.0, 0.0, 0.0, 0.0)
        self.rpm_ref = (0.0, 0.0, 0.0, 0.0)

        # Derniers PWM envoyés
        self.last_duty = [0, 0, 0, 0]

        # Canaux matériels de la carte
        # Normalement :
        # canal 1 -> M1
        # canal 2 -> M2
        # canal 3 -> M3
        # canal 4 -> M4
        self.motor_channels = [1, 2, 3, 4]

        # Signes matériels des moteurs
        #
        # Cette ligne adapte le sens physique réel des moteurs.
        # Ne change pas la cinématique pour corriger un seul moteur.
        # Change seulement cette ligne si une roue tourne toujours à l'envers.
        #
        # Version actuelle selon ton dernier test :
        self.motor_sign = [-1, +1, -1, +1]

        # Si M4 est inversé, utiliser :
        # self.motor_sign = [+1, +1, -1, -1]
        #
        # Si M1 doit revenir comme avant, utiliser :
        # self.motor_sign = [-1, +1, -1, +1]

        if self.enable_debug:
            print("[INIT] MecanumChassis initialized", flush=True)
            print(f"[INIT] motor_channels = {self.motor_channels}", flush=True)
            print(f"[INIT] motor_sign     = {self.motor_sign}", flush=True)
            print(f"[INIT] pwm_limit      = {self.pwm_limit}", flush=True)
            print(f"[INIT] duty_deadzone  = {self.duty_deadzone}", flush=True)
            print(
                f"[INIT] FF RPM->PWM    = offset={self.ff_pwm_offset:.2f}, "
                f"slope={self.ff_pwm_slope:.2f}, "
                f"min={self.ff_pwm_min_moving:.1f}, "
                f"max_ol={self.ff_pwm_max_open_loop:.1f}",
                flush=True
            )

    @staticmethod
    def _rad_per_sec_to_rpm(w_rad_s: float) -> float:
        """Convertit rad/s vers RPM."""
        return float(w_rad_s) * 60.0 / (2.0 * math.pi)

    @staticmethod
    def _rpm_to_rad_per_sec(rpm: float) -> float:
        """Convertit RPM vers rad/s."""
        return float(rpm) * (2.0 * math.pi) / 60.0

    @staticmethod
    def _sign_symbol(value: float, eps: float = 1e-6) -> str:
        """Retourne +, -, ou 0 pour affichage."""
        if value > eps:
            return "+"
        if value < -eps:
            return "-"
        return "0"

    def _clamp_pwm(self, value: int) -> int:
        """Limite le PWM entre -pwm_limit et +pwm_limit."""
        if value > self.pwm_limit:
            return self.pwm_limit
        if value < -self.pwm_limit:
            return -self.pwm_limit
        return value

    def _apply_deadzone(self, value: int) -> int:
        """Force les petits PWM à 0."""
        if abs(value) <= self.duty_deadzone:
            return 0
        return value

    def reset_motors(self) -> None:
        """Arrêter tous les moteurs."""
        self.last_duty = [0, 0, 0, 0]

        if self.enable_debug:
            print("[STOP] reset_motors -> M1=0 M2=0 M3=0 M4=0", flush=True)

        board.set_motor_duty([
            [self.motor_channels[0], 0],
            [self.motor_channels[1], 0],
            [self.motor_channels[2], 0],
            [self.motor_channels[3], 0],
        ])

    def _compute_wheel_linear_speeds_mm_s(
        self,
        velocity_mm_s: float,
        direction_deg: float,
        angular_rate_rad_s: float
    ):
        """
        Cinématique Mecanum.

        Convention :
        vx > 0 : avancer
        vy > 0 : translation gauche
        angular_rate > 0 : rotation selon la convention clavier utilisée
        """
        rad = math.pi / 180.0

        vx = float(velocity_mm_s) * math.cos(float(direction_deg) * rad)
        vy = float(velocity_mm_s) * math.sin(float(direction_deg) * rad)

        # Contribution de la rotation en mm/s
        vp = float(angular_rate_rad_s) * (self.a + self.b)

        # Modèle cinématique Mecanum
        # M1 = avant gauche
        # M2 = avant droite
        # M3 = arrière gauche
        # M4 = arrière droite
        v1 = vx - vy + vp
        v2 = vx + vy - vp
        v3 = vx + vy + vp
        v4 = vx - vy - vp

        return vx, vy, vp, v1, v2, v3, v4

    def _convert_v_to_w_and_rpm(self, v_mm_s: float):
        """
        Conversion :
        w(rad/s) = v(mm/s) / R(mm)
        rpm = w * 60 / (2*pi)
        """
        if self.wheel_radius <= 0.0:
            w = 0.0
        else:
            w = float(v_mm_s) / float(self.wheel_radius)

        rpm = self._rad_per_sec_to_rpm(w)
        return w, rpm

    def _rpm_to_open_loop_pwm(self, rpm_ref: float, mode: str = "translation") -> int:
        """
        Conversion expérimentale RPM_ref -> PWM open-loop.

        mode = "translation" ou "rotation".
        rpm_ref est signé dans la convention cinématique roue.
        Le signe matériel moteur est appliqué ensuite par motor_sign.
        """
        rpm = float(rpm_ref)

        if abs(rpm) < self.rpm_deadband:
            return 0

        if mode == "rotation":
            offset = self.ff_rot_offset
            slope = self.ff_rot_slope
            min_moving = self.ff_rot_min_moving
            max_open_loop = self.ff_rot_max_open_loop
        else:
            offset = self.ff_trans_offset
            slope = self.ff_trans_slope
            min_moving = self.ff_trans_min_moving
            max_open_loop = self.ff_trans_max_open_loop

        duty_abs = offset + slope * abs(rpm)
        duty_abs = max(min_moving, duty_abs)
        duty_abs = min(max_open_loop, duty_abs)

        if rpm > 0:
            return int(round(duty_abs))
        else:
            return -int(round(duty_abs))

    def apply_motor_duty(self, duty1=None, duty2=None, duty3=None, duty4=None) -> None:
        """
        Applique les PWM aux 4 moteurs.

        Si dutyX est None, on conserve la dernière valeur envoyée.
        """
        d1 = self.last_duty[0] if duty1 is None else int(duty1)
        d2 = self.last_duty[1] if duty2 is None else int(duty2)
        d3 = self.last_duty[2] if duty3 is None else int(duty3)
        d4 = self.last_duty[3] if duty4 is None else int(duty4)

        # Limitation PWM
        d1 = self._clamp_pwm(d1)
        d2 = self._clamp_pwm(d2)
        d3 = self._clamp_pwm(d3)
        d4 = self._clamp_pwm(d4)

        self.last_duty = [d1, d2, d3, d4]

        if self.enable_debug:
            print(
                f"[PWM_SEND] "
                f"CH{self.motor_channels[0]}/M1={d1:4d}  "
                f"CH{self.motor_channels[1]}/M2={d2:4d}  "
                f"CH{self.motor_channels[2]}/M3={d3:4d}  "
                f"CH{self.motor_channels[3]}/M4={d4:4d}",
                flush=True
            )

        board.set_motor_duty([
            [self.motor_channels[0], d1],
            [self.motor_channels[1], d2],
            [self.motor_channels[2], d3],
            [self.motor_channels[3], d4],
        ])

    def override_m1_only(self, duty1: int) -> None:
        """Override uniquement M1, conserve M2, M3 et M4."""
        self.apply_motor_duty(duty1=duty1, duty2=None, duty3=None, duty4=None)

    def get_last_duty(self):
        """Retourne les derniers PWM envoyés aux moteurs."""
        return tuple(self.last_duty)

    def set_velocity(
        self,
        velocity: float,
        direction: float,
        angular_rate: float,
        fake: bool = False
    ):
        """
        Convertit velocity, direction et angular_rate vers les commandes moteurs.

        velocity     : mm/s
        direction    : degrés
        angular_rate : rad/s
        fake=True    : calcule sans commander les moteurs
        """
        velocity = float(velocity)
        direction = float(direction)
        angular_rate = float(angular_rate)

        vx, vy, vp, v1, v2, v3, v4 = self._compute_wheel_linear_speeds_mm_s(
            velocity_mm_s=velocity,
            direction_deg=direction,
            angular_rate_rad_s=angular_rate
        )

        # Conversion physique roue : v -> w -> rpm
        w1, rpm1 = self._convert_v_to_w_and_rpm(v1)
        w2, rpm2 = self._convert_v_to_w_and_rpm(v2)
        w3, rpm3 = self._convert_v_to_w_and_rpm(v3)
        w4, rpm4 = self._convert_v_to_w_and_rpm(v4)

        # Mémorisation interne
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate

        self.v_mm_s = (v1, v2, v3, v4)
        self.w_rad_s = (w1, w2, w3, w4)
        self.rpm_ref = (rpm1, rpm2, rpm3, rpm4)

        # PWM open-loop calculé à partir de rpm_ref.
        # On utilise une vraie loi expérimentale RPM_ref -> PWM,
        # au lieu de prendre directement duty = v_roue_mm_s.
        # Choix du modèle feedforward selon le type de mouvement.
        if abs(angular_rate) > 1e-6 and abs(velocity) < 1e-6:
            ff_mode = "rotation"
        else:
            ff_mode = "translation"

        pwm1 = self._rpm_to_open_loop_pwm(rpm1, ff_mode)
        pwm2 = self._rpm_to_open_loop_pwm(rpm2, ff_mode)
        pwm3 = self._rpm_to_open_loop_pwm(rpm3, ff_mode)
        pwm4 = self._rpm_to_open_loop_pwm(rpm4, ff_mode)

        # Application des signes moteurs physiques
        duty1 = int(round(self.motor_sign[0] * pwm1))
        duty2 = int(round(self.motor_sign[1] * pwm2))
        duty3 = int(round(self.motor_sign[2] * pwm3))
        duty4 = int(round(self.motor_sign[3] * pwm4))

        # Deadzone pour forcer les roues supposées arrêtées à 0
        duty1 = self._apply_deadzone(duty1)
        duty2 = self._apply_deadzone(duty2)
        duty3 = self._apply_deadzone(duty3)
        duty4 = self._apply_deadzone(duty4)

        # Limitation PWM
        duty1 = self._clamp_pwm(duty1)
        duty2 = self._clamp_pwm(duty2)
        duty3 = self._clamp_pwm(duty3)
        duty4 = self._clamp_pwm(duty4)

        if self.enable_debug:
            print(
                "\n"
                f"[CMD] velocity={velocity:.2f} mm/s  "
                f"direction={direction:.1f} deg  "
                f"omega={angular_rate:.4f} rad/s",
                flush=True
            )

            print(
                f"[KIN] vx={vx:.2f} mm/s  "
                f"vy={vy:.2f} mm/s  "
                f"vp={vp:.2f} mm/s  "
                f"a+b={self.a + self.b:.2f} mm",
                flush=True
            )

            print(
                f"[V_LOGIC] "
                f"M1={v1:8.2f} ({self._sign_symbol(v1)})  "
                f"M2={v2:8.2f} ({self._sign_symbol(v2)})  "
                f"M3={v3:8.2f} ({self._sign_symbol(v3)})  "
                f"M4={v4:8.2f} ({self._sign_symbol(v4)})",
                flush=True
            )

            print(
                f"[RPM_REF] "
                f"M1={rpm1:8.2f}  "
                f"M2={rpm2:8.2f}  "
                f"M3={rpm3:8.2f}  "
                f"M4={rpm4:8.2f}",
                flush=True
            )

            print(
                f"[FF_MODE] {ff_mode}  "
                f"pwm_logic: M1={pwm1:+d} M2={pwm2:+d} M3={pwm3:+d} M4={pwm4:+d}",
                flush=True
            )

            print(
                f"[MOTOR_SIGN] "
                f"M1={self.motor_sign[0]:+d}  "
                f"M2={self.motor_sign[1]:+d}  "
                f"M3={self.motor_sign[2]:+d}  "
                f"M4={self.motor_sign[3]:+d}",
                flush=True
            )

            print(
                f"[PWM_CALC] "
                f"M1={duty1:4d} ({self._sign_symbol(duty1)})  "
                f"M2={duty2:4d} ({self._sign_symbol(duty2)})  "
                f"M3={duty3:4d} ({self._sign_symbol(duty3)})  "
                f"M4={duty4:4d} ({self._sign_symbol(duty4)})",
                flush=True
            )

        if fake:
            return {
                "inputs": {
                    "velocity_mm_s": velocity,
                    "direction_deg": direction,
                    "omega_robot_rad_s": angular_rate,
                },
                "intermediate": {
                    "vx_mm_s": vx,
                    "vy_mm_s": vy,
                    "vp_mm_s": vp,
                },
                "wheel_linear_mm_s": {
                    "v1": v1,
                    "v2": v2,
                    "v3": v3,
                    "v4": v4,
                },
                "wheel_angular_rad_s": {
                    "w1": w1,
                    "w2": w2,
                    "w3": w3,
                    "w4": w4,
                },
                "wheel_rpm_ref": {
                    "r1": rpm1,
                    "r2": rpm2,
                    "r3": rpm3,
                    "r4": rpm4,
                },
                "motor_sign": {
                    "M1": self.motor_sign[0],
                    "M2": self.motor_sign[1],
                    "M3": self.motor_sign[2],
                    "M4": self.motor_sign[3],
                },
                "pwm_calc": {
                    "M1": duty1,
                    "M2": duty2,
                    "M3": duty3,
                    "M4": duty4,
                },
            }

        self.apply_motor_duty(duty1, duty2, duty3, duty4)

    def translation(self, vx: float, vy: float, fake: bool = False):
        """
        Traduction vx, vy vers velocity, direction puis commande du châssis.

        vx, vy : composantes de vitesse en mm/s
        """
        vx = float(vx)
        vy = float(vy)

        velocity = math.sqrt(vx * vx + vy * vy)

        if vx != 0.0 or vy != 0.0:
            direction = math.degrees(math.atan2(vy, vx))
        else:
            direction = 0.0

        if direction < 0.0:
            direction += 360.0

        if fake:
            return velocity, direction

        self.set_velocity(velocity, direction, 0.0)

    def get_last_rpm_ref(self):
        """Retourne rpm1, rpm2, rpm3, rpm4."""
        return self.rpm_ref

    def get_last_wheel_w_rad_s(self):
        """Retourne w1, w2, w3, w4 en rad/s."""
        return self.w_rad_s

    def get_last_wheel_v_mm_s(self):
        """Retourne v1, v2, v3, v4 en mm/s."""
        return self.v_mm_s
