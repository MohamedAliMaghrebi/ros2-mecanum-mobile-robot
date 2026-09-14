#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class ESP32EncoderReader(Node):
    """
    Lecture ESP32 via USB série.

    Format reçu depuis ESP32 :
    rpm_M1,rpm_M2,rpm_M3,rpm_M4,ticks_M1,ticks_M2,ticks_M3,ticks_M4
    """

    def __init__(self):
        super().__init__("esp32_encoder_reader")

        self.port = "/dev/ttyUSB0"
        self.baud = 115200

        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            "/esp32/wheel_rpm",
            10
        )

        self.ticks_pub = self.create_publisher(
            Int32MultiArray,
            "/esp32/wheel_ticks",
            10
        )

        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        self.get_logger().info("✅ ESP32 encoder reader actif")
        self.get_logger().info(f"✅ Lecture série : {self.port} @ {self.baud}")
        self.get_logger().info("✅ Publication : /esp32/wheel_rpm")
        self.get_logger().info("✅ Publication : /esp32/wheel_ticks")

    def read_serial(self):
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                return

            parts = line.split(",")

            if len(parts) != 8:
                return

            rpm_m1 = float(parts[0])
            rpm_m2 = float(parts[1])
            rpm_m3 = float(parts[2])
            rpm_m4 = float(parts[3])

            ticks_m1 = int(parts[4])
            ticks_m2 = int(parts[5])
            ticks_m3 = int(parts[6])
            ticks_m4 = int(parts[7])

            rpm_msg = Float32MultiArray()
            rpm_msg.data = [rpm_m1, rpm_m2, rpm_m3, rpm_m4]
            self.rpm_pub.publish(rpm_msg)

            ticks_msg = Int32MultiArray()
            ticks_msg.data = [ticks_m1, ticks_m2, ticks_m3, ticks_m4]
            self.ticks_pub.publish(ticks_msg)

            self.get_logger().info(
                f"RPM_MEAS | "
                f"M1={rpm_m1:8.2f}  M2={rpm_m2:8.2f}  "
                f"M3={rpm_m3:8.2f}  M4={rpm_m4:8.2f} || "
                f"TICKS | "
                f"M1={ticks_m1:7d}  M2={ticks_m2:7d}  "
                f"M3={ticks_m3:7d}  M4={ticks_m4:7d}"
            )

        except Exception as e:
            self.get_logger().error(f"Erreur lecture ESP32 : {e}")

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32EncoderReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
