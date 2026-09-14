#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


# ============================================================
# MECANUM ODOMETRY WEB NODE - FINAL WOW DASHBOARD
# DASHBOARD FINAL PRO - PLAN VS TRAJECTOIRE REELLE
# ============================================================
#
# - afficher x, y, theta, vx, vy, wz ;
# - afficher les RPM des 4 moteurs ;
# - afficher la trajectoire estimée ;
# - commander le robot avec les boutons type clavier ;
# - lancer un rectangle avec longueur/largeur réglables ;
# - lancer un cercle avec diamètre réglable ;
# - lancer un triangle avec côté réglable ;
# - STOP prioritaire pour arrêter toute commande.
#
# ============================================================


# ============================================================
# 1. PARAMETRES ODOMETRIE
# ============================================================

WHEEL_RADIUS_M = 0.0325
A_M = 0.175
B_M = 0.115
L_M = A_M + B_M

ODOM_SCALE_X = 2.20
ODOM_SCALE_Y = 2.76
ODOM_SCALE_WZ = 3.60

# ============================================================
# 1.b CORRECTION D'AFFICHAGE DU DASHBOARD
# ============================================================
#
# IMPORTANT : ces coefficients corrigent seulement l'affichage x/y
# dans le dashboard. Ils ne changent pas le mouvement réel du robot
# et ne changent pas le contrôle d'angle theta.
#
# D'après le test rectangle 50 x 70 cm :
# - le robot réel fait le bon rectangle ;
# - l'affichage est trop petit d'environ 1 / 1.40 ;
# - on applique donc un facteur d'affichage uniforme 1.40.
#
# Si plus tard le rectangle affiché mesure par exemple 0.48 m au lieu
# de 0.50 m, ajuster avec :
# DISPLAY_SCALE_X *= 0.50 / 0.48
# DISPLAY_SCALE_Y *= 0.70 / valeur_y_affichee
#
DISPLAY_SCALE_X = 1.40
DISPLAY_SCALE_Y = 1.40


# ============================================================
# 2. PARAMETRES VALIDES DU ROBOT REEL
# ============================================================

CMD_LINEAR = 0.035
CMD_ANGULAR = 0.12

X_TIME = 2.50
X_DISTANCE_REF_CM = 30.0
X_DISTANCE_REF_M = 0.30

Y_TIME = 16.00

WZ_TIME = 3.55
WZ_ANGLE_REF_DEG = 90.0
WZ_ANGLE_REF_RAD = math.pi / 2.0

CIRCLE_TIME_GAIN = 2.50

# Gains expérimentaux après correction PI
# Translation un peu courte -> +10 %
# Rotation trop courte -> +50 %
LINEAR_TIME_GAIN = 1.00
ROT_TIME_GAIN = 1.00


# ============================================================
# 3. PARAMETRES EXECUTION
# ============================================================

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

CMD_PUBLISH_HZ = 50.0
CMD_DT = 1.0 / CMD_PUBLISH_HZ

COMMAND_WATCHDOG_TIMEOUT = 0.60

AUTO_STOP_TIME = 0.50
AUTO_FINAL_STOP_TIME = 1.00

# ============================================================
# 3.b CONTROLE ANGULAIRE PAR THETA + IMPULSIONS
# ============================================================
#
# Principe :
# - pas de tolérance fixe de -8 degrés ;
# - le temps de rotation est seulement une sécurité maximale ;
# - l'angle est suivi avec STATE.theta en temps réel ;
# - avant 90 % de l'angle : rotation continue normale ;
# - après 90 % de l'angle : petites impulsions de rotation ;
# - après chaque impulsion : arrêt court pour laisser theta se stabiliser ;
# - validation si l'erreur est dans +/- 1.5 degré ;
# - aucune correction en sens inverse : si dépassement, on stoppe et on signale.
#
ANGLE_TOL_DEG = 1.0
ANGLE_TOL_RAD = math.radians(ANGLE_TOL_DEG)

ANGLE_PULSE_START_RATIO = 0.90

# Durées des impulsions. Plus l'angle restant est petit, plus l'impulsion est courte.
ANGLE_PULSE_LONG_S = 0.080      # reste > 15 deg
ANGLE_PULSE_MEDIUM_S = 0.055    # reste 7..15 deg
ANGLE_PULSE_FINE_S = 0.035      # reste 3..7 deg
ANGLE_PULSE_ULTRA_FINE_S = 0.025  # reste < 3 deg

# Temps de repos après impulsion pour lire theta stabilisé.
ANGLE_WAIT_LONG_S = 0.060
ANGLE_WAIT_MEDIUM_S = 0.090
ANGLE_WAIT_FINE_S = 0.120
ANGLE_WAIT_ULTRA_FINE_S = 0.150

ANGLE_LOG_PERIOD_S = 0.20

# Sécurité anti-blocage.
# Important : avec le mode impulsionnel, la rotation dure forcément plus longtemps
# que l'ancien WZ_TIME calibré pour une rotation continue. Le temps ci-dessous
# devient donc une vraie sécurité large, pas une condition normale d'arrêt.
ANGLE_TIMEOUT_GAIN = 8.0
ANGLE_TIMEOUT_EXTRA_S = 4.0

# Si theta ne progresse plus pendant cette durée, on arrête pour éviter une boucle infinie.
ANGLE_NO_PROGRESS_TIMEOUT_S = 6.0
ANGLE_PROGRESS_EPS_DEG = 0.25


# ============================================================
# 4. OUTILS DE CALCUL DES TEMPS
# ============================================================

def time_from_distance_cm(distance_cm: float) -> float:
    return X_TIME * float(distance_cm) / X_DISTANCE_REF_CM


def time_from_angle_deg(angle_deg: float) -> float:
    return WZ_TIME * float(angle_deg) / WZ_ANGLE_REF_DEG


def clamp_float(value, vmin, vmax):
    return max(vmin, min(vmax, float(value)))


def make_step(label, vx, vy, wz, duration):
    return {
        "label": str(label),
        "vx": float(vx),
        "vy": float(vy),
        "wz": float(wz),
        "duration": max(0.0, float(duration)),
    }



def angle_delta_signed_rad(current, previous):
    """
    Différence angulaire signée robuste avec wrap [-pi, pi].

    Cette fonction permet d'accumuler une rotation même si theta passe
    de +179 deg à -179 deg. Elle est utilisée pour suivre 90, 120 deg
    ou plus sans perdre l'information de rotation.
    """
    d = float(current) - float(previous)
    return math.atan2(math.sin(d), math.cos(d))


def make_rotation_step(label, wz, target_angle_deg, max_duration, vx=0.0, vy=0.0):
    """
    Étape de rotation contrôlée par theta en temps réel.

    Le temps max_duration est seulement une sécurité.
    L'arrêt normal est décidé par theta :
        angle parcouru ≈ target_angle_deg, avec tolérance +/- ANGLE_TOL_DEG.

    vx et vy sont gardés à 0 pour les rotations de rectangle/triangle.
    """
    nominal_duration = max(0.001, float(max_duration))
    safety_duration = nominal_duration * ANGLE_TIMEOUT_GAIN + ANGLE_TIMEOUT_EXTRA_S

    step = make_step(label, vx, vy, wz, safety_duration)
    step["nominal_duration"] = nominal_duration
    step["safety_duration"] = safety_duration
    step["angle_control"] = True
    step["target_angle_rad"] = math.radians(abs(float(target_angle_deg)))
    step["target_angle_deg"] = abs(float(target_angle_deg))
    step["angle_tolerance_rad"] = ANGLE_TOL_RAD
    return step
def make_stop(label="Pause", duration=AUTO_STOP_TIME):
    return make_step(label, 0.0, 0.0, 0.0, duration)


def build_rectangle_steps(length_cm, width_cm, turn_sign):
    length_cm = clamp_float(length_cm, 5.0, 200.0)
    width_cm = clamp_float(width_cm, 5.0, 200.0)
    turn_sign = 1.0 if turn_sign >= 0 else -1.0

    length_time = time_from_distance_cm(length_cm) * LINEAR_TIME_GAIN
    width_time = time_from_distance_cm(width_cm) * LINEAR_TIME_GAIN
    turn_time = WZ_TIME * ROT_TIME_GAIN

    steps = [
        make_stop("Préparation rectangle", 1.0),
        make_step(f"Rectangle - côté 1 : {length_cm:.1f} cm", CMD_LINEAR, 0.0, 0.0, length_time),
        make_stop("Pause après côté 1"),
        make_rotation_step("Rectangle - rotation 1 : 90 deg", turn_sign * CMD_ANGULAR, 90.0, turn_time),
        make_stop("Pause après rotation 1"),

        make_step(f"Rectangle - côté 2 : {width_cm:.1f} cm", CMD_LINEAR, 0.0, 0.0, width_time),
        make_stop("Pause après côté 2"),
        make_rotation_step("Rectangle - rotation 2 : 90 deg", turn_sign * CMD_ANGULAR, 90.0, turn_time),
        make_stop("Pause après rotation 2"),

        make_step(f"Rectangle - côté 3 : {length_cm:.1f} cm", CMD_LINEAR, 0.0, 0.0, length_time),
        make_stop("Pause après côté 3"),
        make_rotation_step("Rectangle - rotation 3 : 90 deg", turn_sign * CMD_ANGULAR, 90.0, turn_time),
        make_stop("Pause après rotation 3"),

        make_step(f"Rectangle - côté 4 : {width_cm:.1f} cm", CMD_LINEAR, 0.0, 0.0, width_time),
        make_stop("Pause après côté 4"),
        make_rotation_step("Rectangle - rotation 4 : 90 deg", turn_sign * CMD_ANGULAR, 90.0, turn_time),
        make_stop("Fin rectangle", AUTO_FINAL_STOP_TIME),
    ]

    meta = {
        "shape": "rectangle",
        "length_cm": length_cm,
        "width_cm": width_cm,
        "turn_sign": turn_sign,
        "length_time": length_time,
        "width_time": width_time,
        "turn_time": turn_time,
        "linear_time_gain": LINEAR_TIME_GAIN,
        "rot_time_gain": ROT_TIME_GAIN,
    }

    return steps, meta


def build_triangle_steps(side_cm, turn_sign):
    side_cm = clamp_float(side_cm, 5.0, 200.0)
    turn_sign = 1.0 if turn_sign >= 0 else -1.0

    side_time = time_from_distance_cm(side_cm)
    turn_deg = 120.0
    turn_time = time_from_angle_deg(turn_deg)

    steps = [make_stop("Préparation triangle", 1.0)]

    for i in range(1, 4):
        steps.append(make_step(f"Triangle - côté {i} : {side_cm:.1f} cm", CMD_LINEAR, 0.0, 0.0, side_time))
        steps.append(make_stop(f"Pause après côté {i}"))
        steps.append(make_rotation_step(
            f"Triangle - rotation {i} : 120 deg",
            turn_sign * CMD_ANGULAR,
            120.0,
            turn_time
        ))
        steps.append(make_stop(f"Pause après rotation {i}" if i < 3 else "Fin triangle", AUTO_FINAL_STOP_TIME if i == 3 else AUTO_STOP_TIME))

    meta = {
        "shape": "triangle",
        "side_cm": side_cm,
        "turn_sign": turn_sign,
        "side_time": side_time,
        "turn_time": turn_time,
    }

    return steps, meta


def build_circle_steps(diameter_cm, turn_sign):
    """
    Générer un cercle complet contrôlé par theta.

    Principe :
    - vx et wz sont appliqués simultanément ;
    - le rayon est défini par le rapport v / |w| ;
    - l'arrêt normal n'est plus basé sur un temps fixe ;
    - la trajectoire se termine lorsque l'angle accumulé atteint 360 deg ;
    - la stratégie 90/10 est appliquée :
        * mouvement continu jusqu'à 90 % de 360 deg ;
        * impulsions et pauses de lecture theta sur les 10 % finaux.
    """

    diameter_cm = clamp_float(diameter_cm, 10.0, 200.0)
    turn_sign = 1.0 if turn_sign >= 0 else -1.0

    # Conversion du diamètre demandé vers le rayon en mètres.
    radius_m = diameter_cm / 200.0

    # Vitesses réelles issues des calibrations expérimentales.
    v_real_ref = X_DISTANCE_REF_M / X_TIME
    w_real_ref = WZ_ANGLE_REF_RAD / WZ_TIME

    # Relation géométrique du cercle :
    # v = R * |omega|
    v_real_needed = radius_m * abs(w_real_ref)

    # Conversion de la vitesse réelle nécessaire
    # vers une consigne linéaire /cmd_vel.
    cmd_linear_circle = (
        CMD_LINEAR
        * v_real_needed
        / max(1e-6, v_real_ref)
    )

    # Limites de sécurité.
    cmd_linear_circle = clamp_float(
        cmd_linear_circle,
        0.005,
        0.050
    )

    # Un cercle complet correspond à une variation totale
    # d'orientation de 360 degrés.
    target_angle_deg = 360.0

    # Durée nominale calculée pour 360 degrés.
    # Elle ne commande plus l'arrêt normal.
    # make_rotation_step() la transforme en timeout de sécurité large.
    nominal_circle_time = time_from_angle_deg(target_angle_deg)

    steps = [
        make_stop(
            "Préparation cercle contrôlé par theta",
            1.0
        ),

        make_rotation_step(
            label=(
                f"Cercle theta 90/10 - diamètre "
                f"{diameter_cm:.1f} cm"
            ),
            wz=turn_sign * CMD_ANGULAR,
            target_angle_deg=target_angle_deg,
            max_duration=nominal_circle_time,
            vx=cmd_linear_circle,
            vy=0.0
        ),

        make_stop(
            "Fin cercle contrôlé par theta",
            AUTO_FINAL_STOP_TIME
        ),
    ]

    meta = {
        "shape": "circle",
        "diameter_cm": diameter_cm,
        "radius_m": radius_m,
        "turn_sign": turn_sign,
        "target_angle_deg": target_angle_deg,
        "cmd_linear_circle": cmd_linear_circle,
        "cmd_angular_circle": turn_sign * CMD_ANGULAR,
        "nominal_circle_time": nominal_circle_time,
        "control_mode": "theta_90_10",
        "v_real_ref": v_real_ref,
        "w_real_ref": w_real_ref,
        "v_real_needed": v_real_needed,
    }

    return steps, meta


def build_rotation_test_steps(angle_deg, turn_sign):
    """
    Test direct d'un angle sans translation.
    Utile pour valider 45, 90, 120 deg depuis le navigateur :
    /start_rotation?angle=90&turn=1
    """
    angle_deg = clamp_float(angle_deg, 5.0, 360.0)
    turn_sign = 1.0 if turn_sign >= 0 else -1.0
    turn_time = time_from_angle_deg(angle_deg) * 2.0

    steps = [
        make_stop("Préparation rotation test", 0.5),
        make_rotation_step(
            f"Rotation test : {angle_deg:.1f} deg",
            turn_sign * CMD_ANGULAR,
            angle_deg,
            turn_time
        ),
        make_stop("Fin rotation test", AUTO_FINAL_STOP_TIME),
    ]

    meta = {
        "shape": "rotation_test",
        "angle_deg": angle_deg,
        "turn_sign": turn_sign,
        "turn_time": turn_time,
    }

    return steps, meta


# ============================================================
# 5. PAGE HTML
# ============================================================

HTML_PAGE = r"""
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Mecanum Smart Odometry Dashboard</title>

<style>
* { box-sizing: border-box; }

body {
    margin: 0;
    overflow: hidden;
    font-family: "Segoe UI", Arial, sans-serif;
    background: radial-gradient(circle at top, #172554 0%, #020617 72%);
    color: white;
}

#header {
    height: 64px;
    display: flex;
    align-items: center;
    padding: 0 28px;
    background: rgba(2, 6, 23, 0.96);
    border-bottom: 2px solid #38bdf8;
}

#title {
    font-size: 25px;
    font-weight: 900;
    letter-spacing: 0.4px;
}

#subtitle {
    margin-left: 18px;
    color: #93c5fd;
    font-size: 14px;
    font-weight: 700;
}

#status {
    margin-left: auto;
    background: rgba(34, 197, 94, 0.16);
    color: #86efac;
    border: 1px solid #22c55e;
    padding: 8px 14px;
    border-radius: 999px;
    font-weight: 900;
}

#main {
    height: calc(100vh - 64px);
    display: grid;
    grid-template-columns: 1fr 470px;
    gap: 16px;
    padding: 16px;
}

#sceneCard {
    position: relative;
    border-radius: 22px;
    overflow: hidden;
    background: #020617;
    border: 1px solid rgba(56, 189, 248, 0.65);
    box-shadow: 0 24px 70px rgba(0,0,0,0.6);
}

#canvas {
    width: 100%;
    height: 100%;
    display: block;
}

#hud {
    position: absolute;
    left: 18px;
    top: 18px;
    padding: 14px 16px;
    min-width: 305px;
    background: rgba(2, 6, 23, 0.78);
    border: 1px solid rgba(56, 189, 248, 0.45);
    border-radius: 16px;
    backdrop-filter: blur(10px);
    font-family: Consolas, monospace;
    font-size: 13px;
    line-height: 1.55;
}

#legend {
    position: absolute;
    bottom: 18px;
    left: 18px;
    padding: 10px 14px;
    border-radius: 14px;
    background: rgba(2,6,23,0.75);
    border: 1px solid rgba(148,163,184,0.35);
    font-size: 13px;
    color: #cbd5e1;
}

#side {
    border-radius: 22px;
    background: rgba(15, 23, 42, 0.94);
    border: 1px solid rgba(148, 163, 184, 0.35);
    box-shadow: 0 24px 70px rgba(0,0,0,0.55);
    padding: 18px;
    overflow-y: auto;
}

.panelTitle {
    font-size: 20px;
    font-weight: 900;
    margin-bottom: 14px;
    padding-bottom: 10px;
    border-bottom: 2px solid #38bdf8;
}

.box, .proBox {
    background: rgba(30, 41, 59, 0.90);
    border: 1px solid rgba(148, 163, 184, 0.25);
    border-radius: 16px;
    padding: 14px;
    margin-bottom: 14px;
}

.proBox {
    background: linear-gradient(180deg, rgba(15,23,42,0.96), rgba(30,41,59,0.92));
    border: 1px solid rgba(56, 189, 248, 0.45);
}

.row {
    display: flex;
    justify-content: space-between;
    margin: 9px 0;
    font-size: 16px;
}

.label { color: #94a3b8; }
.value { color: #e0f2fe; font-weight: 900; }

button {
    width: 100%;
    border: none;
    border-radius: 13px;
    padding: 12px;
    background: linear-gradient(135deg, #0284c7, #0ea5e9);
    color: white;
    font-size: 15px;
    font-weight: 900;
    cursor: pointer;
    margin-bottom: 8px;
    user-select: none;
    -webkit-user-select: none;
    touch-action: manipulation;
}

button:hover { filter: brightness(1.14); }
button:active { transform: scale(0.97); }

.btnGray { background: linear-gradient(135deg, #475569, #64748b); }
.btnGreen { background: linear-gradient(135deg, #16a34a, #22c55e); }
.btnRed { background: linear-gradient(135deg, #dc2626, #ef4444); }
.btnOrange { background: linear-gradient(135deg, #ea580c, #f97316); }
.btnPurple { background: linear-gradient(135deg, #7c3aed, #a855f7); }

.btnGrid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
}

.note {
    margin-top: 10px;
    color: #94a3b8;
    font-size: 13px;
    line-height: 1.45;
}

#compass {
    width: 100%;
    height: 190px;
    display: block;
}

.teleopStatus {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 10px;
    padding: 10px 12px;
    background: rgba(2,6,23,0.70);
    border: 1px solid rgba(148,163,184,0.28);
    border-radius: 14px;
    font-size: 13px;
}

.badge {
    padding: 6px 10px;
    border-radius: 999px;
    font-weight: 900;
    background: rgba(34,197,94,0.16);
    color: #86efac;
    border: 1px solid #22c55e;
}

.badgeRun {
    background: rgba(249,115,22,0.16);
    color: #fdba74;
    border: 1px solid #f97316;
}

.teleopGrid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 9px;
    margin-bottom: 10px;
}

.teleopBtn {
    height: 58px;
    margin-bottom: 0;
    font-size: 24px;
    border: 1px solid rgba(125, 211, 252, 0.35);
    box-shadow: 0 10px 20px rgba(0,0,0,0.25);
}

.teleopBtn small {
    display: block;
    font-size: 10px;
    margin-top: 2px;
    color: #e0f2fe;
    opacity: 0.9;
}

.teleopStop {
    height: 62px;
    font-size: 18px;
    letter-spacing: 0.7px;
    margin-top: 10px;
}

.rotGrid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 9px;
    margin-top: 8px;
}

.cmdMini {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    margin-top: 10px;
}

.cmdMini div {
    background: rgba(2,6,23,0.65);
    border: 1px solid rgba(148,163,184,0.25);
    border-radius: 12px;
    padding: 8px;
    text-align: center;
    font-family: Consolas, monospace;
    font-size: 12px;
}

.cmdMini span {
    display: block;
    color: #38bdf8;
    font-weight: 900;
    font-size: 13px;
    margin-top: 4px;
}

.formGrid2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
}

.formGrid3 {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 10px;
}

.inputBox {
    margin-bottom: 10px;
}

.inputBox label {
    display: block;
    color: #94a3b8;
    font-size: 13px;
    font-weight: 800;
    margin-bottom: 5px;
}

.inputBox input, .inputBox select {
    width: 100%;
    padding: 10px;
    border-radius: 12px;
    border: 1px solid rgba(148,163,184,0.35);
    background: rgba(2,6,23,0.65);
    color: white;
    font-weight: 900;
}

.trajectoryStatus {
    background: rgba(2,6,23,0.65);
    border: 1px solid rgba(148,163,184,0.25);
    border-radius: 14px;
    padding: 12px;
    margin-top: 10px;
    font-family: Consolas, monospace;
    font-size: 13px;
    line-height: 1.5;
}


.legendLine {
    display: inline-block;
    width: 34px;
    height: 0;
    border-top: 3px solid #38bdf8;
    margin: 0 6px -3px 8px;
}

.legendDash {
    display: inline-block;
    width: 34px;
    height: 0;
    border-top: 3px dashed #facc15;
    margin: 0 6px -3px 8px;
}

.pillRow {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    margin-top: 8px;
}

.pill {
    border-radius: 999px;
    padding: 5px 9px;
    background: rgba(14,165,233,0.14);
    border: 1px solid rgba(56,189,248,0.30);
    color: #bae6fd;
    font-weight: 900;
    font-size: 12px;
}

.progressOuter {
    width: 100%;
    height: 12px;
    background: rgba(15,23,42,0.95);
    border: 1px solid rgba(148,163,184,0.25);
    border-radius: 999px;
    overflow: hidden;
    margin-top: 8px;
}

.progressInner {
    height: 100%;
    width: 0%;
    background: linear-gradient(90deg, #38bdf8, #22c55e);
}
</style>
</head>

<body>
<div id="header">
    <div id="title">Mecanum Robot — Final Experimental Dashboard</div>
    <div id="subtitle">Plan · Real trajectory · Theta control</div>
    <div id="status">LIVE</div>
</div>

<div id="main">
    <div id="sceneCard">
        <canvas id="canvas"></canvas>

        <div id="hud">
            <div><b>ODOMETRY</b></div>
            <div>x      : <span id="hud_x">0.000</span> m</div>
            <div>y      : <span id="hud_y">0.000</span> m</div>
            <div>theta  : <span id="hud_th">0.0</span> deg</div>
            <div>vx     : <span id="hud_vx">0.000</span> m/s</div>
            <div>vy     : <span id="hud_vy">0.000</span> m/s</div>
            <div>wz     : <span id="hud_wz">0.000</span> rad/s</div>
            <div>zoom   : <span id="hud_zoom">---</span> px/m</div>
            <div>mode   : <span id="hud_mode">AUTO-FIT</span></div>
            <div>cmd    : <span id="hud_cmd">STOP</span></div>
            <div>traj   : <span id="hud_traj">IDLE</span></div>
        </div>

        <div id="legend">
            <span class="legendDash"></span>Trajectoire demandée · <span class="legendLine"></span>Trajectoire réelle estimée · Rouge : centre robot · Flèche blanche : orientation
        </div>
    </div>

    <div id="side">
        <div class="panelTitle">Commande robot</div>

        <div class="proBox">
            <div class="teleopStatus">
                <div>
                    <b>Téléopération</b><br>
                    <span style="color:#94a3b8;">Appui maintenu = mouvement</span>
                </div>
                <div class="badge" id="teleopBadge">READY</div>
            </div>

            <div class="teleopGrid">
                <button class="teleopBtn btnPurple" data-vx="0.03" data-vy="0.03" data-wz="0" data-label="Q · Diag avant gauche">↖<small>q</small></button>
                <button class="teleopBtn" data-vx="0.03" data-vy="0" data-wz="0" data-label="W · Avant">↑<small>w</small></button>
                <button class="teleopBtn btnPurple" data-vx="0.03" data-vy="-0.03" data-wz="0" data-label="E · Diag avant droite">↗<small>e</small></button>

                <button class="teleopBtn" data-vx="0" data-vy="0.03" data-wz="0" data-label="A · Translation gauche">←<small>a</small></button>
                <button class="teleopBtn btnRed" data-vx="0" data-vy="0" data-wz="0" data-label="X · Stop">■<small>x</small></button>
                <button class="teleopBtn" data-vx="0" data-vy="-0.03" data-wz="0" data-label="D · Translation droite">→<small>d</small></button>

                <button class="teleopBtn btnPurple" data-vx="-0.03" data-vy="0.03" data-wz="0" data-label="Z · Diag arrière gauche">↙<small>z</small></button>
                <button class="teleopBtn" data-vx="-0.03" data-vy="0" data-wz="0" data-label="S · Arrière">↓<small>s</small></button>
                <button class="teleopBtn btnPurple" data-vx="-0.03" data-vy="-0.03" data-wz="0" data-label="C · Diag arrière droite">↘<small>c</small></button>
            </div>

            <div class="rotGrid">
                <button class="btnOrange teleopBtn" data-vx="0" data-vy="0" data-wz="-0.12" data-label="F · Rotation antihoraire">⟲<small>f</small></button>
                <button class="btnOrange teleopBtn" data-vx="0" data-vy="0" data-wz="0.12" data-label="R · Rotation horaire">⟳<small>r</small></button>
            </div>

            <button class="btnRed teleopStop" onclick="emergencyStop()">STOP URGENCE</button>

            <div class="cmdMini">
                <div>vx<span id="cmd_vx">0.000</span></div>
                <div>vy<span id="cmd_vy">0.000</span></div>
                <div>wz<span id="cmd_wz">0.000</span></div>
            </div>
        </div>

        <div class="panelTitle">Trajectoires automatiques</div>

        <div class="proBox">
            <div class="teleopStatus">
                <div>
                    <b>Trajectoire</b><br>
                    <span style="color:#94a3b8;">Choisir la forme et les dimensions</span>
                </div>
                <div class="badge" id="trajBadge">IDLE</div>
            </div>

            <div class="inputBox">
                <label>Rectangle — longueur / largeur [cm]</label>
                <div class="formGrid2">
                    <input id="rect_length" type="number" value="30" min="5" max="200">
                    <input id="rect_width" type="number" value="20" min="5" max="200">
                </div>
            </div>

            <div class="btnGrid">
                <button class="btnGreen" onclick="startRectangle()">Lancer rectangle</button>
                <button class="btnGray" onclick="resetOdom()">Reset odom</button>
            </div>

            <div class="inputBox">
                <label>Cercle — diamètre [cm]</label>
                <input id="circle_diameter" type="number" value="40" min="10" max="200">
            </div>

            <button class="btnGreen" onclick="startCircle()">Lancer cercle</button>

            <div class="inputBox">
                <label>Triangle — côté [cm]</label>
                <input id="triangle_side" type="number" value="30" min="5" max="200">
            </div>

            <button class="btnGreen" onclick="startTriangle()">Lancer triangle</button>

            <div class="inputBox">
                <label>Sens de rotation</label>
                <select id="turn_sign">
                    <option value="1">Horaire WZ+</option>
                    <option value="-1">Antihoraire WZ-</option>
                </select>
            </div>

            <button class="btnRed" onclick="stopTrajectory()">STOP trajectoire</button>

            <div class="trajectoryStatus">
                <div>État : <span id="traj_state">IDLE</span></div>
                <div>Étape : <span id="traj_step">---</span></div>
                <div>Progression : <span id="traj_progress_text">0%</span></div>
                <div style="margin-top:8px; color:#e0f2fe;"><b>Contrôle angle theta</b></div>
                <div>Phase : <span id="angle_phase">---</span></div>
                <div>Cible : <span id="angle_target">---</span> deg</div>
                <div>Mesuré : <span id="angle_measured">---</span> deg</div>
                <div>Reste : <span id="angle_remaining">---</span> deg</div>
                <div>Erreur : <span id="angle_error">---</span> deg</div>
                <div class="progressOuter">
                    <div class="progressInner" id="traj_progress"></div>
                </div>
            </div>
        </div>

        <div class="panelTitle">État estimé</div>

        <div class="box">
            <div class="row"><span class="label">x</span><span class="value" id="x">0.000 m</span></div>
            <div class="row"><span class="label">y</span><span class="value" id="y">0.000 m</span></div>
            <div class="row"><span class="label">θ</span><span class="value" id="theta">0.0°</span></div>
        </div>

        <div class="box">
            <div class="row"><span class="label">vx</span><span class="value" id="vx">0.000 m/s</span></div>
            <div class="row"><span class="label">vy</span><span class="value" id="vy">0.000 m/s</span></div>
            <div class="row"><span class="label">ωz</span><span class="value" id="wz">0.000 rad/s</span></div>
        </div>

        <div class="box">
            <div class="row"><span class="label">RPM M1</span><span class="value" id="m1">0.0</span></div>
            <div class="row"><span class="label">RPM M2</span><span class="value" id="m2">0.0</span></div>
            <div class="row"><span class="label">RPM M3</span><span class="value" id="m3">0.0</span></div>
            <div class="row"><span class="label">RPM M4</span><span class="value" id="m4">0.0</span></div>
        </div>

        <div class="panelTitle">Vue intelligente</div>

        <div class="btnGrid">
            <button onclick="setViewMode('auto')">Auto-fit</button>
            <button onclick="setViewMode('follow')">Follow robot</button>
            <button onclick="zoomIn()">Zoom +</button>
            <button onclick="zoomOut()">Zoom -</button>
        </div>

        <button class="btnGray" onclick="resetView()">Reset vue</button>
        <button class="btnRed" onclick="resetOdom()">Reset odométrie</button>

        <div class="panelTitle" style="margin-top:18px;">Orientation</div>

        <div class="box">
            <canvas id="compass"></canvas>
            <div class="row"><span class="label">Angle</span><span class="value" id="compassTheta">0.0°</span></div>
        </div>
    </div>
</div>

<script>
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

const compass = document.getElementById("compass");
const cctx = compass.getContext("2d");

let W = 0;
let H = 0;

let viewMode = "auto";
let ppm = 260;
let centerWorldX = 0;
let centerWorldY = 0;

let manualPpm = 260;
let manualCenterX = 0;
let manualCenterY = 0;

let commandInterval = null;
let pressedKey = null;

let previewShape = null;
let previewAnchor = null;
let planActive = false;
let planParams = null;
let lastState = null;
let plannedPath = [];

// Interaction visuelle robot : quand la souris est sur le robot,
// le châssis devient semi-transparent pour voir exactement le centre et la trajectoire derrière.
let mouseCanvasX = null;
let mouseCanvasY = null;
let robotHover = false;

function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    W = rect.width;
    H = rect.height;
    canvas.width = W;
    canvas.height = H;

    const crect = compass.getBoundingClientRect();
    compass.width = crect.width;
    compass.height = crect.height;
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();

canvas.addEventListener("mousemove", (e) => {
    const rect = canvas.getBoundingClientRect();
    mouseCanvasX = e.clientX - rect.left;
    mouseCanvasY = e.clientY - rect.top;
});

canvas.addEventListener("mouseleave", () => {
    mouseCanvasX = null;
    mouseCanvasY = null;
    robotHover = false;
    canvas.style.cursor = "default";
});

function setViewMode(mode) { viewMode = mode; }

function zoomIn() {
    viewMode = "manual";
    manualPpm *= 1.25;
}

function zoomOut() {
    viewMode = "manual";
    manualPpm /= 1.25;
}

function resetView() {
    viewMode = "auto";
    manualPpm = 260;
    manualCenterX = 0;
    manualCenterY = 0;
}

async function resetOdom() {
    try { await fetch("/stop"); } catch (e) { console.log("Erreur stop avant reset:", e); }
    try { await fetch("/reset"); } catch (e) { console.log("Erreur reset odom:", e); }

    previewShape = null;
    previewAnchor = null;
    planActive = false;
    planParams = null;
    plannedPath = [];

    document.getElementById("angle_phase").innerText = "---";
    document.getElementById("angle_target").innerText = "---";
    document.getElementById("angle_measured").innerText = "---";
    document.getElementById("angle_remaining").innerText = "---";
    document.getElementById("angle_error").innerText = "---";
    document.getElementById("traj_state").innerText = "IDLE";
    document.getElementById("traj_step").innerText = "---";
    document.getElementById("traj_progress_text").innerText = "0%";
    document.getElementById("traj_progress").style.width = "0%";
    document.getElementById("hud_traj").innerText = "IDLE";

    resetView();
}

async function sendCmd(vx, vy, wz, label="CMD") {
    document.getElementById("cmd_vx").innerText = vx.toFixed(3);
    document.getElementById("cmd_vy").innerText = vy.toFixed(3);
    document.getElementById("cmd_wz").innerText = wz.toFixed(3);
    document.getElementById("teleopBadge").innerText = label === "STOP" ? "STOP" : "MOVING";
    document.getElementById("hud_cmd").innerText = label;

    const url = `/cmd?vx=${encodeURIComponent(vx)}&vy=${encodeURIComponent(vy)}&wz=${encodeURIComponent(wz)}&label=${encodeURIComponent(label)}`;

    try { await fetch(url); } catch (e) { console.log("Erreur sendCmd:", e); }
}

function startCommand(vx, vy, wz, label) {
    stopCommandInterval();
    sendCmd(vx, vy, wz, label);
    commandInterval = setInterval(() => {
        sendCmd(vx, vy, wz, label);
    }, 180);
}

function stopCommandInterval() {
    if (commandInterval !== null) {
        clearInterval(commandInterval);
        commandInterval = null;
    }
}

function stopCommand() {
    stopCommandInterval();
    sendCmd(0.0, 0.0, 0.0, "STOP");
}

async function emergencyStop() {
    stopCommandInterval();
    await fetch("/stop");
    sendCmd(0.0, 0.0, 0.0, "STOP URGENCE");
}

function attachTeleopButtons() {
    const buttons = document.querySelectorAll(".teleopBtn");

    buttons.forEach(btn => {
        const vx = parseFloat(btn.dataset.vx);
        const vy = parseFloat(btn.dataset.vy);
        const wz = parseFloat(btn.dataset.wz);
        const label = btn.dataset.label || "CMD";

        btn.addEventListener("mousedown", (e) => {
            e.preventDefault();
            startCommand(vx, vy, wz, label);
        });

        btn.addEventListener("mouseup", (e) => {
            e.preventDefault();
            stopCommand();
        });

        btn.addEventListener("mouseleave", (e) => {
            e.preventDefault();
            stopCommand();
        });

        btn.addEventListener("touchstart", (e) => {
            e.preventDefault();
            startCommand(vx, vy, wz, label);
        }, {passive: false});

        btn.addEventListener("touchend", (e) => {
            e.preventDefault();
            stopCommand();
        }, {passive: false});

        btn.addEventListener("touchcancel", (e) => {
            e.preventDefault();
            stopCommand();
        }, {passive: false});
    });
}

function keyToCommand(key) {
    const v = 0.03;
    const w = 0.12;

    const map = {
        "w": [ v,  0,  0, "W · Avant"],
        "s": [-v,  0,  0, "S · Arrière"],
        "a": [ 0,  v,  0, "A · Gauche"],
        "d": [ 0, -v,  0, "D · Droite"],
        "q": [ v,  v,  0, "Q · Diag avant gauche"],
        "e": [ v, -v,  0, "E · Diag avant droite"],
        "z": [-v,  v,  0, "Z · Diag arrière gauche"],
        "c": [-v, -v,  0, "C · Diag arrière droite"],
        "r": [ 0,  0,  w, "R · Rotation horaire"],
        "f": [ 0,  0, -w, "F · Rotation antihoraire"],
        "x": [ 0,  0,  0, "STOP"]
    };

    return map[key] || null;
}

window.addEventListener("keydown", (e) => {
    const key = e.key.toLowerCase();
    if (pressedKey === key) return;

    const cmd = keyToCommand(key);
    if (!cmd) return;

    e.preventDefault();
    pressedKey = key;

    if (key === "x") {
        emergencyStop();
    } else {
        startCommand(cmd[0], cmd[1], cmd[2], cmd[3]);
    }
});

window.addEventListener("keyup", (e) => {
    const key = e.key.toLowerCase();
    if (pressedKey === key) {
        e.preventDefault();
        pressedKey = null;
        stopCommand();
    }
});

window.addEventListener("blur", () => { stopCommand(); });

attachTeleopButtons();

function getTurnSign() {
    return parseFloat(document.getElementById("turn_sign").value);
}

function clampNumber(v, fallback) {
    const x = parseFloat(v);
    return Number.isFinite(x) ? x : fallback;
}

function capturePreviewAnchor() {
    if (lastState) {
        previewAnchor = {x: lastState.x, y: lastState.y, theta: lastState.theta};
    }
}

function armPlan(shape, params) {
    previewShape = shape;
    planParams = params;
    planActive = true;
    capturePreviewAnchor();
}

function addForwardPoint(points, heading, distanceM) {
    const last = points[points.length - 1];
    points.push([
        last[0] + Math.cos(heading) * distanceM,
        last[1] + Math.sin(heading) * distanceM
    ]);
}

function buildRectanglePreview(anchor) {
    const params = planParams || {length: 30, width: 20, turn: 1};
    const L = clampNumber(params.length, 30) / 100.0;
    const Wd = clampNumber(params.width, 20) / 100.0;
    const turn = clampNumber(params.turn, 1);
    let h = anchor.theta;
    const pts = [[anchor.x, anchor.y]];
    addForwardPoint(pts, h, L); h += turn * Math.PI / 2;
    addForwardPoint(pts, h, Wd); h += turn * Math.PI / 2;
    addForwardPoint(pts, h, L); h += turn * Math.PI / 2;
    addForwardPoint(pts, h, Wd);
    return pts;
}

function buildTrianglePreview(anchor) {
    const params = planParams || {side: 30, turn: 1};
    const side = clampNumber(params.side, 30) / 100.0;
    const turn = clampNumber(params.turn, 1);
    let h = anchor.theta;
    const pts = [[anchor.x, anchor.y]];
    for (let i = 0; i < 3; i++) {
        addForwardPoint(pts, h, side);
        h += turn * 2 * Math.PI / 3;
    }
    return pts;
}

function buildCirclePreview(anchor) {
    const params = planParams || {diameter: 40, turn: 1};
    const diameter = clampNumber(params.diameter, 40) / 100.0;
    const r = diameter / 2.0;
    const turn = clampNumber(params.turn, 1);
    const nx = -Math.sin(anchor.theta);
    const ny = Math.cos(anchor.theta);
    const cx = anchor.x + turn * nx * r;
    const cy = anchor.y + turn * ny * r;
    const startAngle = Math.atan2(anchor.y - cy, anchor.x - cx);
    const pts = [];
    for (let i = 0; i <= 96; i++) {
        const a = startAngle - turn * 2 * Math.PI * i / 96;
        pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
    }
    return pts;
}

function buildPlannedPath(s) {
    if (!planActive || !previewAnchor || !previewShape || !planParams) return [];
    const anchor = previewAnchor;
    if (previewShape === "rectangle") return buildRectanglePreview(anchor);
    if (previewShape === "triangle") return buildTrianglePreview(anchor);
    if (previewShape === "circle") return buildCirclePreview(anchor);
    return [];
}

function extents(path) {
    if (!path || path.length < 2) return null;
    let minX = path[0][0], maxX = path[0][0], minY = path[0][1], maxY = path[0][1];
    for (const p of path) {
        minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
        minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    }
    return {x: maxX - minX, y: maxY - minY};
}

function updatePreviewPanel(s) { /* Panel supprimé dans la version finale PRO. */ }

async function startRectangle() {
    const length = parseFloat(document.getElementById("rect_length").value);
    const width = parseFloat(document.getElementById("rect_width").value);
    const turn = getTurnSign();
    armPlan("rectangle", {length, width, turn});

    await fetch(`/start_rectangle?length=${length}&width=${width}&turn=${turn}`);
}

async function startCircle() {
    const diameter = parseFloat(document.getElementById("circle_diameter").value);
    const turn = getTurnSign();
    armPlan("circle", {diameter, turn});

    await fetch(`/start_circle?diameter=${diameter}&turn=${turn}`);
}

async function startTriangle() {
    const side = parseFloat(document.getElementById("triangle_side").value);
    const turn = getTurnSign();
    armPlan("triangle", {side, turn});

    await fetch(`/start_triangle?side=${side}&turn=${turn}`);
}

async function stopTrajectory() {
    await fetch("/trajectory_stop");
}

function computeView(s) {
    const path = s.path || [[0, 0]];
    plannedPath = buildPlannedPath(s);

    if (viewMode === "follow") {
        centerWorldX = s.x;
        centerWorldY = s.y;
        ppm = 420;
        return;
    }

    if (viewMode === "manual") {
        centerWorldX = manualCenterX;
        centerWorldY = manualCenterY;
        ppm = manualPpm;
        return;
    }

    let minX = s.x, maxX = s.x;
    let minY = s.y, maxY = s.y;

    const allViewPoints = path.concat(plannedPath || []);
    for (let i = 0; i < allViewPoints.length; i++) {
        const x = allViewPoints[i][0];
        const y = allViewPoints[i][1];

        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
    }

    // Zoom intelligent :
    // - si un plan est lancé, la caméra cadre directement le dessin demandé + la trajectoire réelle ;
    // - si aucun plan n'est lancé, on garde une vue plus large autour du robot.
    const hasPlan = planActive && plannedPath && plannedPath.length > 1;
    const marginM = hasPlan ? 0.16 : 0.55;
    minX -= marginM;
    maxX += marginM;
    minY -= marginM;
    maxY += marginM;

    const minSpan = hasPlan ? 0.55 : 1.0;
    const spanX = Math.max(maxX - minX, minSpan);
    const spanY = Math.max(maxY - minY, minSpan);

    const ppmX = W / spanX;
    const ppmY = H / spanY;

    ppm = Math.min(ppmX, ppmY);
    ppm = hasPlan ? Math.max(120, Math.min(ppm, 820)) : Math.max(70, Math.min(ppm, 420));

    centerWorldX = (minX + maxX) / 2.0;
    centerWorldY = (minY + maxY) / 2.0;
}

function worldToCanvas(x, y) {
    return [
        W / 2 + (x - centerWorldX) * ppm,
        H / 2 - (y - centerWorldY) * ppm
    ];
}

function drawBackground() {
    const grad = ctx.createLinearGradient(0, 0, 0, H);
    grad.addColorStop(0, "#0f172a");
    grad.addColorStop(1, "#020617");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, W, H);
}

function niceGridStep() {
    const targetPx = 95;
    const raw = targetPx / ppm;
    const powers = [0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 5];

    for (let i = 0; i < powers.length; i++) {
        if (powers[i] >= raw) return powers[i];
    }
    return 10;
}

function drawGrid() {
    drawBackground();

    const step = niceGridStep();
    const minor = step / 2.0;

    const leftWorld = centerWorldX - W / (2 * ppm);
    const rightWorld = centerWorldX + W / (2 * ppm);
    const bottomWorld = centerWorldY - H / (2 * ppm);
    const topWorld = centerWorldY + H / (2 * ppm);

    const startX = Math.floor(leftWorld / minor) * minor;
    const endX = Math.ceil(rightWorld / minor) * minor;
    const startY = Math.floor(bottomWorld / minor) * minor;
    const endY = Math.ceil(topWorld / minor) * minor;

    for (let x = startX; x <= endX; x += minor) {
        const p1 = worldToCanvas(x, bottomWorld);
        const p2 = worldToCanvas(x, topWorld);
        const major = Math.abs((x / step) - Math.round(x / step)) < 1e-6;

        ctx.strokeStyle = major ? "rgba(148,163,184,0.30)" : "rgba(148,163,184,0.12)";
        ctx.lineWidth = major ? 1.2 : 1.0;
        ctx.beginPath();
        ctx.moveTo(p1[0], p1[1]);
        ctx.lineTo(p2[0], p2[1]);
        ctx.stroke();

        if (major && ppm > 90) {
            const p = worldToCanvas(x, 0);
            ctx.fillStyle = "rgba(203,213,225,0.65)";
            ctx.font = "12px Consolas";
            ctx.fillText(x.toFixed(2), p[0] + 4, H - 12);
        }
    }

    for (let y = startY; y <= endY; y += minor) {
        const p1 = worldToCanvas(leftWorld, y);
        const p2 = worldToCanvas(rightWorld, y);
        const major = Math.abs((y / step) - Math.round(y / step)) < 1e-6;

        ctx.strokeStyle = major ? "rgba(148,163,184,0.30)" : "rgba(148,163,184,0.12)";
        ctx.lineWidth = major ? 1.2 : 1.0;
        ctx.beginPath();
        ctx.moveTo(p1[0], p1[1]);
        ctx.lineTo(p2[0], p2[1]);
        ctx.stroke();

        if (major && ppm > 90) {
            const p = worldToCanvas(0, y);
            ctx.fillStyle = "rgba(203,213,225,0.65)";
            ctx.font = "12px Consolas";
            ctx.fillText(y.toFixed(2), 8, p[1] - 4);
        }
    }

    ctx.strokeStyle = "rgba(56,189,248,0.85)";
    ctx.lineWidth = 2.2;

    let p1 = worldToCanvas(leftWorld, 0);
    let p2 = worldToCanvas(rightWorld, 0);
    ctx.beginPath();
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.stroke();

    p1 = worldToCanvas(0, bottomWorld);
    p2 = worldToCanvas(0, topWorld);
    ctx.beginPath();
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.stroke();

    ctx.fillStyle = "rgba(226,232,240,0.90)";
    ctx.font = "bold 14px Segoe UI";
    ctx.fillText("x [m]", W - 70, worldToCanvas(0, 0)[1] - 10);
    ctx.fillText("y [m]", worldToCanvas(0, 0)[0] + 12, 28);
}

function drawPath(path) {
    if (!path || path.length < 2) return;

    ctx.lineJoin = "round";
    ctx.lineCap = "round";

    ctx.strokeStyle = "rgba(56,189,248,0.22)";
    ctx.lineWidth = 12;
    ctx.beginPath();

    let p = worldToCanvas(path[0][0], path[0][1]);
    ctx.moveTo(p[0], p[1]);

    for (let i = 1; i < path.length; i++) {
        p = worldToCanvas(path[i][0], path[i][1]);
        ctx.lineTo(p[0], p[1]);
    }

    ctx.stroke();

    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 4;
    ctx.beginPath();

    p = worldToCanvas(path[0][0], path[0][1]);
    ctx.moveTo(p[0], p[1]);

    for (let i = 1; i < path.length; i++) {
        p = worldToCanvas(path[i][0], path[i][1]);
        ctx.lineTo(p[0], p[1]);
    }

    ctx.stroke();
}

function drawPlannedPath(path) {
    if (!path || path.length < 2) return;

    ctx.save();
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.setLineDash([14, 10]);

    ctx.strokeStyle = "rgba(250,204,21,0.20)";
    ctx.lineWidth = 10;
    ctx.beginPath();
    let p = worldToCanvas(path[0][0], path[0][1]);
    ctx.moveTo(p[0], p[1]);
    for (let i = 1; i < path.length; i++) {
        p = worldToCanvas(path[i][0], path[i][1]);
        ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke();

    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 3.0;
    ctx.beginPath();
    p = worldToCanvas(path[0][0], path[0][1]);
    ctx.moveTo(p[0], p[1]);
    for (let i = 1; i < path.length; i++) {
        p = worldToCanvas(path[i][0], path[i][1]);
        ctx.lineTo(p[0], p[1]);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    const pStart = worldToCanvas(path[0][0], path[0][1]);
    const pEnd = worldToCanvas(path[path.length - 1][0], path[path.length - 1][1]);
    ctx.fillStyle = "#facc15";
    ctx.beginPath(); ctx.arc(pStart[0], pStart[1], 6, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = "#facc15";
    ctx.beginPath(); ctx.arc(pEnd[0], pEnd[1], 5, 0, 2*Math.PI); ctx.fill();
    ctx.restore();
}

function roundedRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
}

function drawRobot(x, y, theta) {
    // Design final plus professionnel :
    // - corps plus compact et arrondi ;
    // - roues visibles ;
    // - flèche d'orientation plus propre ;
    // - point central toujours visible ;
    // - transparence au survol pour voir la trajectoire sous le robot.
    const L = Math.max(54, Math.min(0.20 * ppm, 118));
    const B = Math.max(38, Math.min(0.135 * ppm, 78));

    const p = worldToCanvas(x, y);

    const hoverRadius = Math.max(L, B) * 0.72;
    const isHover = (
        mouseCanvasX !== null &&
        mouseCanvasY !== null &&
        Math.hypot(mouseCanvasX - p[0], mouseCanvasY - p[1]) <= hoverRadius
    );

    robotHover = isHover;
    canvas.style.cursor = isHover ? "crosshair" : "default";

    // Centre réel du robot : dessiné avant le corps, puis redessiné après.
    // Quand le robot est transparent, cela permet de voir précisément où il est arrivé.
    ctx.save();
    ctx.fillStyle = isHover ? "rgba(239,68,68,0.18)" : "rgba(239,68,68,0.10)";
    ctx.beginPath();
    ctx.arc(p[0], p[1], isHover ? 18 : 11, 0, 2*Math.PI);
    ctx.fill();
    ctx.restore();

    ctx.save();
    ctx.translate(p[0], p[1]);
    ctx.rotate(-theta);

    // Ombre douce
    ctx.globalAlpha = isHover ? 0.22 : 1.0;
    ctx.fillStyle = "rgba(0,0,0,0.42)";
    roundedRect(ctx, -L/2 + 7, -B/2 + 8, L, B, 14);
    ctx.fill();

    // Corps principal
    const bodyGrad = ctx.createLinearGradient(-L/2, -B/2, L/2, B/2);
    bodyGrad.addColorStop(0, "rgba(226,232,240,0.96)");
    bodyGrad.addColorStop(0.55, "rgba(148,163,184,0.92)");
    bodyGrad.addColorStop(1, "rgba(71,85,105,0.88)");

    roundedRect(ctx, -L/2, -B/2, L, B, 14);
    ctx.fillStyle = bodyGrad;
    ctx.fill();
    ctx.strokeStyle = "rgba(248,250,252,0.95)";
    ctx.lineWidth = 2.6;
    ctx.stroke();

    // Roues Mecanum stylisées
    const wheelW = L * 0.19;
    const wheelH = B * 0.23;
    const wheelX = L * 0.30;
    const wheelY = B * 0.36;

    function drawWheel(cx, cy, angle) {
        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(angle);
        roundedRect(ctx, -wheelW/2, -wheelH/2, wheelW, wheelH, 5);
        ctx.fillStyle = "rgba(15,23,42,0.92)";
        ctx.fill();
        ctx.strokeStyle = "rgba(203,213,225,0.62)";
        ctx.lineWidth = 1.2;
        ctx.stroke();

        ctx.strokeStyle = "rgba(125,211,252,0.55)";
        ctx.lineWidth = 1.0;
        for (let k = -2; k <= 2; k++) {
            ctx.beginPath();
            ctx.moveTo(k * wheelW / 5 - wheelW * 0.22, -wheelH * 0.28);
            ctx.lineTo(k * wheelW / 5 + wheelW * 0.22,  wheelH * 0.28);
            ctx.stroke();
        }
        ctx.restore();
    }

    drawWheel( wheelX,  wheelY,  Math.PI/10);
    drawWheel( wheelX, -wheelY, -Math.PI/10);
    drawWheel(-wheelX,  wheelY, -Math.PI/10);
    drawWheel(-wheelX, -wheelY,  Math.PI/10);

    // Module central
    roundedRect(ctx, -L*0.22, -B*0.24, L*0.44, B*0.48, 8);
    ctx.fillStyle = "rgba(15,23,42,0.82)";
    ctx.fill();
    ctx.strokeStyle = "rgba(148,163,184,0.45)";
    ctx.lineWidth = 1.1;
    ctx.stroke();

    // Flèche d'orientation avant
    const arrowStart = L * 0.06;
    const arrowEnd = L * 0.58;
    ctx.strokeStyle = "rgba(255,255,255,0.96)";
    ctx.lineWidth = Math.max(4.0, Math.min(6.5, L * 0.06));
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(arrowStart, 0);
    ctx.lineTo(arrowEnd, 0);
    ctx.stroke();

    ctx.fillStyle = "rgba(255,255,255,0.98)";
    ctx.beginPath();
    ctx.moveTo(arrowEnd + L*0.14, 0);
    ctx.lineTo(arrowEnd - L*0.02, -B*0.26);
    ctx.lineTo(arrowEnd - L*0.02,  B*0.26);
    ctx.closePath();
    ctx.fill();

    // Contour spécifique au survol
    if (isHover) {
        ctx.globalAlpha = 1.0;
        ctx.setLineDash([7, 5]);
        roundedRect(ctx, -L/2 - 3, -B/2 - 3, L + 6, B + 6, 16);
        ctx.strokeStyle = "rgba(250,204,21,0.95)";
        ctx.lineWidth = 2.0;
        ctx.stroke();
        ctx.setLineDash([]);
    }

    ctx.restore();

    // Centre robot précis : toujours au-dessus du corps
    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,0.96)";
    ctx.lineWidth = isHover ? 2.4 : 1.8;
    ctx.beginPath();
    ctx.moveTo(p[0] - 10, p[1]);
    ctx.lineTo(p[0] + 10, p[1]);
    ctx.moveTo(p[0], p[1] - 10);
    ctx.lineTo(p[0], p[1] + 10);
    ctx.stroke();

    ctx.fillStyle = "#ef4444";
    ctx.beginPath();
    ctx.arc(p[0], p[1], isHover ? 5.5 : 4.5, 0, 2*Math.PI);
    ctx.fill();

    if (isHover) {
        ctx.font = "bold 12px Consolas";
        ctx.fillStyle = "rgba(226,232,240,0.96)";
        ctx.fillText(`centre (${x.toFixed(3)}, ${y.toFixed(3)}) m`, p[0] + 14, p[1] - 14);
    }
    ctx.restore();
}

function drawCompass(theta) {
    const cw = compass.width;
    const ch = compass.height;
    const cx = cw / 2;
    const cy = ch / 2;
    const r = Math.min(cw, ch) * 0.36;

    cctx.clearRect(0, 0, cw, ch);

    cctx.strokeStyle = "rgba(148,163,184,0.35)";
    cctx.lineWidth = 2;
    cctx.beginPath();
    cctx.arc(cx, cy, r, 0, 2*Math.PI);
    cctx.stroke();

    cctx.fillStyle = "rgba(15,23,42,0.8)";
    cctx.beginPath();
    cctx.arc(cx, cy, r - 4, 0, 2*Math.PI);
    cctx.fill();

    cctx.fillStyle = "#94a3b8";
    cctx.font = "bold 13px Segoe UI";
    cctx.fillText("0°", cx + r + 8, cy + 4);
    cctx.fillText("90°", cx - 12, cy - r - 8);
    cctx.fillText("180°", cx - r - 42, cy + 4);
    cctx.fillText("-90°", cx - 16, cy + r + 20);

    cctx.save();
    cctx.translate(cx, cy);
    cctx.rotate(-theta);

    cctx.strokeStyle = "#ffffff";
    cctx.lineWidth = 6;
    cctx.beginPath();
    cctx.moveTo(0, 0);
    cctx.lineTo(r * 0.82, 0);
    cctx.stroke();

    cctx.fillStyle = "#ef4444";
    cctx.beginPath();
    cctx.arc(0, 0, 6, 0, 2*Math.PI);
    cctx.fill();

    cctx.fillStyle = "#ffffff";
    cctx.beginPath();
    cctx.moveTo(r * 0.96, 0);
    cctx.lineTo(r * 0.70, -12);
    cctx.lineTo(r * 0.70, 12);
    cctx.closePath();
    cctx.fill();

    cctx.restore();
}

async function update() {
    const r = await fetch("/state");
    const s = await r.json();
    lastState = s;

    computeView(s);
    updatePreviewPanel(s);

    const thetaDeg = s.theta * 180 / Math.PI;

    document.getElementById("x").innerText = s.x.toFixed(3) + " m";
    document.getElementById("y").innerText = s.y.toFixed(3) + " m";
    document.getElementById("theta").innerText = thetaDeg.toFixed(1) + "°";
    document.getElementById("vx").innerText = s.vx.toFixed(3) + " m/s";
    document.getElementById("vy").innerText = s.vy.toFixed(3) + " m/s";
    document.getElementById("wz").innerText = s.wz.toFixed(3) + " rad/s";

    document.getElementById("m1").innerText = s.rpm[0].toFixed(1);
    document.getElementById("m2").innerText = s.rpm[1].toFixed(1);
    document.getElementById("m3").innerText = s.rpm[2].toFixed(1);
    document.getElementById("m4").innerText = s.rpm[3].toFixed(1);

    document.getElementById("hud_x").innerText = s.x.toFixed(3);
    document.getElementById("hud_y").innerText = s.y.toFixed(3);
    document.getElementById("hud_th").innerText = thetaDeg.toFixed(1);
    document.getElementById("hud_vx").innerText = s.vx.toFixed(3);
    document.getElementById("hud_vy").innerText = s.vy.toFixed(3);
    document.getElementById("hud_wz").innerText = s.wz.toFixed(3);
    document.getElementById("hud_zoom").innerText = ppm.toFixed(0);
    document.getElementById("hud_mode").innerText = viewMode.toUpperCase();

    if (s.cmd) {
        document.getElementById("cmd_vx").innerText = s.cmd.vx.toFixed(3);
        document.getElementById("cmd_vy").innerText = s.cmd.vy.toFixed(3);
        document.getElementById("cmd_wz").innerText = s.cmd.wz.toFixed(3);
    }

    if (s.trajectory) {
        const t = s.trajectory;
        const running = t.running;
        const pct = Math.round((t.progress || 0) * 100);

        document.getElementById("trajBadge").innerText = running ? "RUNNING" : "IDLE";
        document.getElementById("trajBadge").className = running ? "badge badgeRun" : "badge";
        document.getElementById("traj_state").innerText = running ? t.name : "IDLE";
        document.getElementById("traj_step").innerText = t.current_label || "---";
        document.getElementById("traj_progress_text").innerText = pct + "%";
        document.getElementById("traj_progress").style.width = pct + "%";
        document.getElementById("hud_traj").innerText = running ? t.name : "IDLE";

        if (t.angle_debug && running) {
            const a = t.angle_debug;
            document.getElementById("angle_phase").innerText = a.phase || "---";
            document.getElementById("angle_target").innerText =
                (a.target_deg === null || a.target_deg === undefined) ? "---" : a.target_deg.toFixed(1);
            document.getElementById("angle_measured").innerText =
                (a.measured_deg === null || a.measured_deg === undefined) ? "---" : a.measured_deg.toFixed(1);
            document.getElementById("angle_remaining").innerText =
                (a.remaining_deg === null || a.remaining_deg === undefined) ? "---" : a.remaining_deg.toFixed(1);
            document.getElementById("angle_error").innerText =
                (a.error_deg === null || a.error_deg === undefined) ? "---" : a.error_deg.toFixed(1);
        } else if (!running) {
            document.getElementById("angle_phase").innerText = "---";
            document.getElementById("angle_target").innerText = "---";
            document.getElementById("angle_measured").innerText = "---";
            document.getElementById("angle_remaining").innerText = "---";
            document.getElementById("angle_error").innerText = "---";
        }
    }

    document.getElementById("compassTheta").innerText = thetaDeg.toFixed(1) + "°";

    drawGrid();
    drawPlannedPath(plannedPath);
    drawPath(s.path);
    drawRobot(s.x, s.y, s.theta);
    drawCompass(s.theta);
}

setInterval(update, 35);
</script>
</body>
</html>
"""


# ============================================================
# 6. ETATS PARTAGES
# ============================================================

class OdomState:
    def __init__(self):
        self.lock = threading.Lock()
        self.rpm = [0.0, 0.0, 0.0, 0.0]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.path = [[0.0, 0.0]]
        self.last_time = time.time()

    def reset(self):
        with self.lock:
            self.rpm = [0.0, 0.0, 0.0, 0.0]
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.path = [[0.0, 0.0]]
            self.last_time = time.time()

    def to_dict(self):
        with self.lock:
            # Correction uniquement pour l'affichage du dashboard.
            # STATE.x / STATE.y restent les valeurs internes brutes ;
            # theta reste inchangé pour conserver le contrôle angulaire validé.
            display_path = [
                [p[0] * DISPLAY_SCALE_X, p[1] * DISPLAY_SCALE_Y]
                for p in self.path[-50000:]
            ]

            return {
                "rpm": self.rpm,

                # Valeurs affichées corrigées
                "x": self.x * DISPLAY_SCALE_X,
                "y": self.y * DISPLAY_SCALE_Y,
                "theta": self.theta,
                "vx": self.vx * DISPLAY_SCALE_X,
                "vy": self.vy * DISPLAY_SCALE_Y,
                "wz": self.wz,
                "path": display_path,

                # Valeurs brutes disponibles pour diagnostic si besoin
                "raw_x": self.x,
                "raw_y": self.y,
                "raw_vx": self.vx,
                "raw_vy": self.vy,
                "display_scale_x": DISPLAY_SCALE_X,
                "display_scale_y": DISPLAY_SCALE_Y,
            }


class TeleopState:
    def __init__(self):
        self.lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.label = "STOP"
        self.active = False
        self.last_http_time = 0.0
        self.stop_pending = True

    def set_command(self, vx, vy, wz, label="CMD"):
        with self.lock:
            self.vx = float(vx)
            self.vy = float(vy)
            self.wz = float(wz)
            self.label = str(label)
            self.active = abs(self.vx) > 1e-9 or abs(self.vy) > 1e-9 or abs(self.wz) > 1e-9
            self.last_http_time = time.time()
            self.stop_pending = not self.active

    def stop(self, label="STOP"):
        with self.lock:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.label = str(label)
            self.active = False
            self.last_http_time = time.time()
            self.stop_pending = True

    def watchdog_stop_if_needed(self):
        with self.lock:
            if self.active and (time.time() - self.last_http_time) > COMMAND_WATCHDOG_TIMEOUT:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.label = "WATCHDOG STOP"
                self.active = False
                self.stop_pending = True
                return True
            return False

    def get_snapshot(self):
        with self.lock:
            return {
                "vx": self.vx,
                "vy": self.vy,
                "wz": self.wz,
                "label": self.label,
                "active": self.active,
                "stop_pending": self.stop_pending,
            }

    def clear_stop_pending(self):
        with self.lock:
            self.stop_pending = False

    def to_dict(self):
        with self.lock:
            return {
                "vx": self.vx,
                "vy": self.vy,
                "wz": self.wz,
                "label": self.label,
                "active": self.active,
            }


class TrajectoryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.name = "IDLE"
        self.steps = []
        self.meta = {}
        self.index = 0
        self.step_start_time = 0.0
        self.step_start_theta = 0.0
        self.current_label = "---"
        self.progress = 0.0
        self.stop_pending = False

        # Etat interne du contrôle angulaire
        self.angle_initialized = False
        self.angle_last_theta = 0.0
        self.angle_progress_rad = 0.0
        self.angle_pulse_on_until = 0.0
        self.angle_wait_until = 0.0
        self.angle_last_log_time = 0.0
        self.angle_last_progress_check_rad = 0.0
        self.angle_last_progress_check_time = time.time()
        self.angle_debug = {
            "phase": "---",
            "target_deg": None,
            "measured_deg": None,
            "remaining_deg": None,
            "error_deg": None,
        }

    def reset_angle_state(self):
        self.angle_initialized = False
        self.angle_last_theta = 0.0
        self.angle_progress_rad = 0.0
        self.angle_pulse_on_until = 0.0
        self.angle_wait_until = 0.0
        self.angle_last_log_time = 0.0
        self.angle_last_progress_check_rad = 0.0
        self.angle_last_progress_check_time = time.time()
        self.angle_debug = {
            "phase": "---",
            "target_deg": None,
            "measured_deg": None,
            "remaining_deg": None,
            "error_deg": None,
        }

    def start(self, name, steps, meta=None):
        with self.lock:
            self.running = True
            self.name = str(name)
            self.steps = list(steps)
            self.meta = dict(meta or {})
            self.index = 0
            self.step_start_time = time.time()
            self.step_start_theta = get_current_theta()
            self.current_label = self.steps[0]["label"] if self.steps else "---"
            self.progress = 0.0
            self.stop_pending = False
            self.reset_angle_state()

    def cancel(self):
        with self.lock:
            self.running = False
            self.name = "STOPPED"
            self.steps = []
            self.meta = {}
            self.index = 0
            self.step_start_time = 0.0
            self.step_start_theta = 0.0
            self.current_label = "STOP trajectoire"
            self.progress = 0.0
            self.stop_pending = True
            self.reset_angle_state()

    def make_zero_step(self, label="STOP"):
        return make_step(label, 0.0, 0.0, 0.0, 0.0)

    def finish_current_step(self, now, phase="ANGLE OK"):
        self.index += 1
        self.step_start_time = now
        self.step_start_theta = get_current_theta()
        self.current_label = phase
        self.reset_angle_state()

    def select_pulse_timing(self, remaining_deg):
        """
        Choisir la durée d'impulsion selon l'angle restant.
        On garde la même vitesse angulaire commandée, mais on réduit
        l'énergie envoyée par des impulsions plus courtes.
        """
        remaining_deg = abs(float(remaining_deg))

        if remaining_deg > 15.0:
            return ANGLE_PULSE_LONG_S, ANGLE_WAIT_LONG_S, "PULSE LONG"

        if remaining_deg > 7.0:
            return ANGLE_PULSE_MEDIUM_S, ANGLE_WAIT_MEDIUM_S, "PULSE MEDIUM"

        if remaining_deg > 3.0:
            return ANGLE_PULSE_FINE_S, ANGLE_WAIT_FINE_S, "PULSE FINE"

        return ANGLE_PULSE_ULTRA_FINE_S, ANGLE_WAIT_ULTRA_FINE_S, "PULSE ULTRA"

    def update_angle_debug(self, phase, target_rad, progress_rad, remaining_rad):
        self.angle_debug = {
            "phase": str(phase),
            "target_deg": math.degrees(float(target_rad)),
            "measured_deg": math.degrees(float(progress_rad)),
            "remaining_deg": math.degrees(float(remaining_rad)),
            "error_deg": math.degrees(float(remaining_rad)),
        }

    def update_angle_step(self, step, now, elapsed):
        """
        Contrôle angle basé sur theta temps réel.

        Phase 1 : avant 90 % de l'angle, rotation continue.
        Phase 2 : après 90 %, impulsions courtes + arrêt + relecture theta.
        Validation : erreur <= +/- 1.5 deg.
        Interdiction : aucune correction en sens inverse.
        """

        current_theta = get_current_theta()

        target_rad = float(step.get("target_angle_rad", 0.0))
        target_rad = max(0.0, target_rad)

        if target_rad <= 0.0:
            self.finish_current_step(now, "ANGLE cible nulle")
            return self.make_zero_step("STOP")

        turn_sign = 1.0 if float(step.get("wz", 0.0)) >= 0.0 else -1.0

        if not self.angle_initialized:
            self.angle_initialized = True
            self.angle_last_theta = current_theta
            self.angle_progress_rad = 0.0
            self.angle_pulse_on_until = 0.0
            self.angle_wait_until = 0.0
            self.angle_last_log_time = 0.0
            self.angle_last_progress_check_rad = 0.0
            self.angle_last_progress_check_time = now

        # Accumulation robuste de l'angle depuis la dernière lecture.
        dtheta = angle_delta_signed_rad(current_theta, self.angle_last_theta)
        self.angle_last_theta = current_theta

        # On accumule seulement dans le sens demandé.
        # Un petit bruit inverse ne doit pas effacer l'angle déjà parcouru.
        dtheta_in_command_direction = turn_sign * dtheta
        if dtheta_in_command_direction > 0.0:
            self.angle_progress_rad += dtheta_in_command_direction

        remaining_rad = target_rad - self.angle_progress_rad
        remaining_deg = math.degrees(remaining_rad)

        # Progression globale affichée
        local_progress = min(1.0, max(0.0, self.angle_progress_rad / target_rad))
        self.progress = (self.index + local_progress) / max(1, len(self.steps))

        # Si l'angle est atteint dans la tolérance : validation.
        if abs(remaining_rad) <= ANGLE_TOL_RAD:
            self.update_angle_debug("OK +/- %.1f deg" % ANGLE_TOL_DEG, target_rad, self.angle_progress_rad, remaining_rad)
            self.finish_current_step(now, "ANGLE OK")
            return self.make_zero_step("STOP ANGLE OK")

        # Si dépassement hors tolérance : on ne revient PAS en arrière.
        # On arrête toute la trajectoire pour ne pas démarrer le prochain segment
        # avec un mauvais angle.
        if remaining_rad < -ANGLE_TOL_RAD:
            self.update_angle_debug("DEPASSEMENT > TOL - STOP TRAJ", target_rad, self.angle_progress_rad, remaining_rad)
            self.running = False
            self.name = "ANGLE_ERROR"
            self.current_label = (
                f"ANGLE DEPASSE : theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | erreur={math.degrees(remaining_rad):.1f} deg"
            )
            self.progress = (self.index + 1.0) / max(1, len(self.steps))
            self.stop_pending = True
            return self.make_zero_step("STOP ANGLE ERROR")

        # Sécurité anti-blocage : on vérifie que theta progresse vraiment.
        # Si aucune progression mesurable n'est observée pendant plusieurs secondes,
        # on arrête la trajectoire. Sinon, on laisse continuer les impulsions.
        progress_eps_rad = math.radians(ANGLE_PROGRESS_EPS_DEG)
        if (self.angle_progress_rad - self.angle_last_progress_check_rad) >= progress_eps_rad:
            self.angle_last_progress_check_rad = self.angle_progress_rad
            self.angle_last_progress_check_time = now
        elif (now - self.angle_last_progress_check_time) >= ANGLE_NO_PROGRESS_TIMEOUT_S:
            self.update_angle_debug("NO THETA PROGRESS - STOP TRAJ", target_rad, self.angle_progress_rad, remaining_rad)
            self.running = False
            self.name = "ANGLE_NO_PROGRESS"
            self.current_label = (
                f"NO THETA PROGRESS : theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f} deg"
            )
            self.progress = (self.index + local_progress) / max(1, len(self.steps))
            self.stop_pending = True
            return self.make_zero_step("STOP NO PROGRESS")

        # Sécurité temporelle large.
        # Avec le mode impulsionnel, l'ancien WZ_TIME n'est plus une durée normale,
        # mais seulement une base pour calculer un timeout très large.
        duration = max(0.001, float(step["duration"]))
        if elapsed >= duration:
            self.update_angle_debug("TIMEOUT LARGE - STOP TRAJ", target_rad, self.angle_progress_rad, remaining_rad)
            self.running = False
            self.name = "ANGLE_TIMEOUT"
            self.current_label = (
                f"TIMEOUT LARGE : theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f} deg"
            )
            self.progress = (self.index + local_progress) / max(1, len(self.steps))
            self.stop_pending = True
            return self.make_zero_step("STOP TIMEOUT")

        # Avant 90 % de l'angle : rotation continue normale.
        if self.angle_progress_rad < ANGLE_PULSE_START_RATIO * target_rad:
            phase = "CONTINU avant 90%"
            self.update_angle_debug(phase, target_rad, self.angle_progress_rad, remaining_rad)
            self.current_label = (
                f"{step['label']} | {phase} | "
                f"theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f}"
            )
            return step

        # Après 90 % : impulsions.
        # 1) Si on est dans une impulsion active, on continue à envoyer wz.
        if now < self.angle_pulse_on_until:
            phase = "IMPULSION ON"
            self.update_angle_debug(phase, target_rad, self.angle_progress_rad, remaining_rad)
            self.current_label = (
                f"{step['label']} | {phase} | "
                f"theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f}"
            )
            return step

        # 2) Si on attend après impulsion, on envoie STOP pour lire theta stabilisé.
        if now < self.angle_wait_until:
            phase = "LECTURE THETA"
            self.update_angle_debug(phase, target_rad, self.angle_progress_rad, remaining_rad)
            self.current_label = (
                f"{step['label']} | {phase} | "
                f"theta={math.degrees(self.angle_progress_rad):.1f}/"
                f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f}"
            )
            return self.make_zero_step("WAIT THETA")

        # 3) Nouvelle impulsion.
        pulse_s, wait_s, phase = self.select_pulse_timing(remaining_deg)
        self.angle_pulse_on_until = now + pulse_s
        self.angle_wait_until = self.angle_pulse_on_until + wait_s

        self.update_angle_debug(phase, target_rad, self.angle_progress_rad, remaining_rad)
        self.current_label = (
            f"{step['label']} | {phase} {pulse_s*1000:.0f} ms | "
            f"theta={math.degrees(self.angle_progress_rad):.1f}/"
            f"{math.degrees(target_rad):.1f} deg | reste={remaining_deg:.1f}"
        )
        return step

    def update_and_get_command(self):
        with self.lock:
            if not self.running:
                return None

            if not self.steps or self.index >= len(self.steps):
                self.running = False
                self.current_label = "DONE"
                self.progress = 1.0
                self.stop_pending = True
                return self.make_zero_step("STOP")

            now = time.time()

            while self.index < len(self.steps):
                step = self.steps[self.index]
                duration = max(0.001, float(step["duration"]))
                elapsed = now - self.step_start_time

                if step.get("angle_control", False):
                    return self.update_angle_step(step, now, elapsed)

                if elapsed < duration:
                    local_progress = elapsed / duration
                    self.progress = (self.index + local_progress) / max(1, len(self.steps))
                    self.current_label = step["label"]
                    self.angle_debug = {
                        "phase": "---",
                        "target_deg": None,
                        "measured_deg": None,
                        "remaining_deg": None,
                        "error_deg": None,
                    }
                    return step

                self.index += 1
                self.step_start_time = now
                self.step_start_theta = get_current_theta()
                self.reset_angle_state()

            self.running = False
            self.current_label = "DONE"
            self.progress = 1.0
            self.stop_pending = True
            return self.make_zero_step("STOP")

    def get_snapshot(self):
        with self.lock:
            return {
                "running": self.running,
                "name": self.name,
                "current_label": self.current_label,
                "index": self.index,
                "total_steps": len(self.steps),
                "progress": self.progress,
                "meta": self.meta,
                "stop_pending": self.stop_pending,
                "angle_debug": dict(self.angle_debug),
            }

    def clear_stop_pending(self):
        with self.lock:
            self.stop_pending = False

STATE = OdomState()


def get_current_theta():
    with STATE.lock:
        return float(STATE.theta)


TELEOP = TeleopState()
TRAJECTORY = TrajectoryState()


# ============================================================
# 7. SERVEUR HTTP
# ============================================================

class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/":
            content = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            self.wfile.write(content)

        elif path == "/state":
            payload = STATE.to_dict()
            payload["cmd"] = TELEOP.to_dict()
            payload["trajectory"] = TRAJECTORY.get_snapshot()

            content = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        elif path == "/reset":
            STATE.reset()
            self.send_ok({"ok": True, "message": "odometry reset"})

        elif path == "/stop":
            TELEOP.stop("STOP")
            TRAJECTORY.cancel()
            self.send_ok({"ok": True, "message": "stop all"})

        elif path == "/trajectory_stop":
            TRAJECTORY.cancel()
            TELEOP.stop("STOP")
            self.send_ok({"ok": True, "message": "trajectory stopped"})

        elif path == "/cmd":
            query = parse_qs(parsed.query)

            try:
                vx = float(query.get("vx", ["0.0"])[0])
                vy = float(query.get("vy", ["0.0"])[0])
                wz = float(query.get("wz", ["0.0"])[0])
                label = query.get("label", ["CMD"])[0]
            except Exception:
                vx, vy, wz, label = 0.0, 0.0, 0.0, "BAD CMD"

            TELEOP.set_command(vx, vy, wz, label)
            self.send_ok({"ok": True, "cmd": TELEOP.to_dict()})

        elif path == "/start_rectangle":
            query = parse_qs(parsed.query)
            length = float(query.get("length", ["30"])[0])
            width = float(query.get("width", ["20"])[0])
            turn = float(query.get("turn", ["1"])[0])

            TELEOP.stop("STOP")
            steps, meta = build_rectangle_steps(length, width, turn)
            TRAJECTORY.start("RECTANGLE", steps, meta)
            self.send_ok({"ok": True, "trajectory": TRAJECTORY.get_snapshot()})

        elif path == "/start_circle":
            query = parse_qs(parsed.query)
            diameter = float(query.get("diameter", ["40"])[0])
            turn = float(query.get("turn", ["1"])[0])

            TELEOP.stop("STOP")
            steps, meta = build_circle_steps(diameter, turn)
            TRAJECTORY.start("CIRCLE", steps, meta)
            self.send_ok({"ok": True, "trajectory": TRAJECTORY.get_snapshot()})

        elif path == "/start_rotation":
            query = parse_qs(parsed.query)
            angle = float(query.get("angle", ["90"])[0])
            turn = float(query.get("turn", ["1"])[0])

            TELEOP.stop("STOP")
            steps, meta = build_rotation_test_steps(angle, turn)
            TRAJECTORY.start("ROTATION_TEST", steps, meta)
            self.send_ok({"ok": True, "trajectory": TRAJECTORY.get_snapshot()})

        elif path == "/start_triangle":
            query = parse_qs(parsed.query)
            side = float(query.get("side", ["30"])[0])
            turn = float(query.get("turn", ["1"])[0])

            TELEOP.stop("STOP")
            steps, meta = build_triangle_steps(side, turn)
            TRAJECTORY.start("TRIANGLE", steps, meta)
            self.send_ok({"ok": True, "trajectory": TRAJECTORY.get_snapshot()})

        else:
            self.send_response(404)
            self.end_headers()

    def send_ok(self, payload):
        content = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format, *args):
        return


def start_web_server():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), WebHandler)
    server.serve_forever()


# ============================================================
# 8. NOEUD ROS 2
# ============================================================

class MecanumOdometryWebNode(Node):
    def __init__(self):
        super().__init__("mecanum_odometry_web_node")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            Float32MultiArray,
            "/esp32/wheel_rpm",
            self.rpm_callback,
            10
        )

        self.odom_timer = self.create_timer(0.02, self.update_odometry)
        self.cmd_timer = self.create_timer(CMD_DT, self.publish_command_manager)

        thread = threading.Thread(target=start_web_server, daemon=True)
        thread.start()

        self.get_logger().info("✅ Smart odometry dashboard FINAL WOW actif")
        self.get_logger().info("✅ Dashboard + téléopération + trajectoires automatiques")
        self.get_logger().info("✅ Rotation theta par impulsions après 90% de l'angle")
        self.get_logger().info(f"✅ Tolérance angle = +/- {ANGLE_TOL_DEG:.1f} deg")
        self.get_logger().info(f"✅ Timeout angle = nominal*{ANGLE_TIMEOUT_GAIN:.1f}+{ANGLE_TIMEOUT_EXTRA_S:.1f}s")
        self.get_logger().info(f"✅ Anti-blocage theta = {ANGLE_NO_PROGRESS_TIMEOUT_S:.1f}s sans progression")
        self.get_logger().info(f"✅ CMD_PUBLISH_HZ = {CMD_PUBLISH_HZ:.1f} Hz")
        self.get_logger().info("✅ Publie /cmd_vel")
        self.get_logger().info(f"✅ Ouvre : http://<IP_RASPBERRY>:{HTTP_PORT}")

    def rpm_callback(self, msg):
        if len(msg.data) >= 4:
            with STATE.lock:
                STATE.rpm = [float(msg.data[i]) for i in range(4)]

    @staticmethod
    def rpm_to_speed(rpm):
        return WHEEL_RADIUS_M * rpm * 2.0 * math.pi / 60.0

    def publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_command_manager(self):
        # Priorité 1 : trajectoire automatique
        traj_step = TRAJECTORY.update_and_get_command()

        if traj_step is not None:
            self.publish_twist(traj_step["vx"], traj_step["vy"], traj_step["wz"])
            return

        traj_snapshot = TRAJECTORY.get_snapshot()

        if traj_snapshot["stop_pending"]:
            self.publish_twist(0.0, 0.0, 0.0)
            TRAJECTORY.clear_stop_pending()
            return

        # Priorité 2 : téléopération web
        TELEOP.watchdog_stop_if_needed()
        snap = TELEOP.get_snapshot()

        if snap["active"]:
            self.publish_twist(snap["vx"], snap["vy"], snap["wz"])

        elif snap["stop_pending"]:
            self.publish_twist(0.0, 0.0, 0.0)
            TELEOP.clear_stop_pending()

    def update_odometry(self):
        now = time.time()

        with STATE.lock:
            dt = now - STATE.last_time
            STATE.last_time = now

            if dt <= 0.0 or dt > 0.2:
                return

            v1 = self.rpm_to_speed(STATE.rpm[0])
            v2 = self.rpm_to_speed(STATE.rpm[1])
            v3 = self.rpm_to_speed(STATE.rpm[2])
            v4 = self.rpm_to_speed(STATE.rpm[3])

            vx = ODOM_SCALE_X * (v1 + v2 + v3 + v4) / 4.0
            vy = ODOM_SCALE_Y * (-v1 + v2 + v3 - v4) / 4.0
            wz = ODOM_SCALE_WZ * (v1 - v2 + v3 - v4) / (4.0 * L_M)

            STATE.vx = vx
            STATE.vy = vy
            STATE.wz = wz

            STATE.x += (vx * math.cos(STATE.theta) - vy * math.sin(STATE.theta)) * dt
            STATE.y += (vx * math.sin(STATE.theta) + vy * math.cos(STATE.theta)) * dt
            STATE.theta += wz * dt
            STATE.theta = math.atan2(math.sin(STATE.theta), math.cos(STATE.theta))

            STATE.path.append([STATE.x, STATE.y])

    def destroy_node(self):
        try:
            self.publish_twist(0.0, 0.0, 0.0)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryWebNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
