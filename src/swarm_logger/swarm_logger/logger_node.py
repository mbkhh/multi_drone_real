#!/usr/bin/env python3

import os
import re
import subprocess
import time
from collections import defaultdict, deque
from threading import Lock

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    ManualControlSetpoint,
    OffboardControlMode,
    SensorGps,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleControlMode,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header, String
from swarm_config.config_utils import get_config
from swarm_msgs.msg import FormationCommand, ManualControl, Status


class SwarmLogger(Node):
    """Collects multi-drone DDS message rates and monitors Raspberry Pi power, thermal, and Wi-Fi health."""

    def __init__(self):
        super().__init__('swarm_logger')

        default_drone_count = int(get_config('swarm_sim.drone_count'))
        self.declare_parameter('drone_count', default_drone_count)
        self.declare_parameter('print_interval', 1.0)
        self.declare_parameter('wifi_interface', 'wlan0')

        self.drone_count = (
            self.get_parameter('drone_count').get_parameter_value().integer_value
        )
        self.print_interval = (
            self.get_parameter('print_interval').get_parameter_value().double_value
        )
        self.wifi_interface = (
            self.get_parameter('wifi_interface').get_parameter_value().string_value
        )

        self._queues = defaultdict(deque)
        self._queue_lock = Lock()
        self._subscriptions = []
        self._last_print_time = time.time()

        self._px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._add_swarm_topics()
        self._add_vehicle_topics()

        self.create_timer(self.print_interval, self._print_and_clear_queues)
        self.get_logger().info(
            f'Diagnostic Logger active for {self.drone_count} drones. Printing every {self.print_interval:.1f}s.'
        )

    def _subscribe(self, message_type, topic, qos):
        callback = lambda message, topic_name=topic: self._enqueue(
            topic_name, message
        )
        sub = self.create_subscription(message_type, topic, callback, qos)
        self._subscriptions.append(sub)

    def _add_swarm_topics(self):
        topics = [
            (Status, '/swarm/status'),
            (Header, '/swarm/live'),
            (String, '/swarm/command'),
            (FormationCommand, '/swarm/formation_command'),
            (ManualControl, '/manual_controller'),
            (String, '/command'),
            (FormationCommand, '/formation'),
        ]
        for message_type, topic in topics:
            self._subscribe(message_type, topic, self._ros_qos)

    def _add_vehicle_topics(self):
        px4_topics = [
            (VehicleStatus, 'vehicle_status_v1'),
            (VehicleLocalPosition, 'vehicle_local_position_v1'),
            (VehicleLandDetected, 'vehicle_land_detected'),
            (BatteryStatus, 'battery_status_v1'),
            (FailsafeFlags, 'failsafe_flags'),
        ]
        control_topics = [
            (VehicleCommand, 'vehicle_command'),
            (TrajectorySetpoint, 'trajectory_setpoint'),
            (OffboardControlMode, 'offboard_control_mode'),
        ]

        for drone_id in range(1, self.drone_count + 1):
            ns = f'/uav_{drone_id}/fmu'
            for message_type, topic_name in px4_topics:
                self._subscribe(message_type, f'{ns}/out/{topic_name}', self._px4_qos)
            for message_type, topic_name in control_topics:
                self._subscribe(message_type, f'{ns}/in/{topic_name}', self._px4_qos)

    def _enqueue(self, topic, message):
        with self._queue_lock:
            self._queues[topic].append(message)

    # ------------------ Hardware Diagnostics ------------------

    def _read_pi_throttled_state(self):
        """Decode vcgencmd get_throttled bitmask for under-voltage and frequency throttling."""
        try:
            res = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=0.2)
            raw_hex = res.stdout.strip().split('=')[-1]
            val = int(raw_hex, 16)
            
            reasons = []
            if val & 0x1:
                reasons.append("CURRENT UNDER-VOLTAGE DETECTED (<4.63V)")
            if val & 0x2:
                reasons.append("CURRENT ARM FREQUENCY CAPPED")
            if val & 0x4:
                reasons.append("CURRENTLY THROTTLED (Thermal/Power)")
            if val & 0x8:
                reasons.append("CURRENT SOFT TEMP LIMIT REACHED")
            if val & 0x10000:
                reasons.append("Under-voltage occurred since boot")
            if val & 0x20000:
                reasons.append("Frequency capping occurred since boot")
            if val & 0x40000:
                reasons.append("Throttling occurred since boot")
            if val & 0x80000:
                reasons.append("Soft temp limit occurred since boot")

            status_str = "HEALTHY (0x0)" if val == 0 else f"ISSUES FOUND ({raw_hex}): " + "; ".join(reasons)
            return status_str, val
        except Exception:
            return "N/A (Non-Raspberry Pi or vcgencmd unavailable)", 0

    def _read_system_metrics(self):
        """Read CPU temperature and load average."""
        temp_str = "N/A"
        try:
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    temp_str = f"{float(f.read().strip()) / 1000.0:.1f}°C"
        except Exception:
            pass

        try:
            load1, load5, _ = os.getloadavg()
            load_str = f"1m={load1:.2f}, 5m={load5:.2f}"
        except Exception:
            load_str = "N/A"

        return temp_str, load_str

    def _read_wifi_link(self):
        """Read signal level, link quality, and power saving status."""
        stats = {"link": "N/A", "signal": "N/A", "power_save": "N/A"}
        try:
            # Check /proc/net/wireless
            if os.path.exists("/proc/net/wireless"):
                with open("/proc/net/wireless", "r") as f:
                    lines = f.readlines()
                    for line in lines:
                        if self.wifi_interface in line:
                            parts = line.split()
                            stats["link"] = parts[2].replace('.', '')
                            stats["signal"] = f"{parts[3].replace('.', '')} dBm"

            # Check power management via iw
            res = subprocess.run(['iw', 'dev', self.wifi_interface, 'get', 'power_save'], capture_output=True, text=True, timeout=0.2)
            if "Power save: on" in res.stdout:
                stats["power_save"] = "CRITICAL: ENABLED (Will drop UDP packets!)"
            elif "Power save: off" in res.stdout:
                stats["power_save"] = "OFF (Optimal)"
        except Exception:
            pass
        return stats

    # ------------------ Logging Output ------------------

    def _print_and_clear_queues(self):
        now = time.time()
        elapsed = now - self._last_print_time
        self._last_print_time = now

        with self._queue_lock:
            snapshot = {topic: len(q) for topic, q in self._queues.items()}
            self._queues.clear()

        throttled_str, throttled_val = self._read_pi_throttled_state()
        temp_str, load_str = self._read_system_metrics()
        wifi_stats = self._read_wifi_link()

        border = "=" * 80
        lines = [
            f"\n{border}",
            f" [SWARM SYSTEM & NETWORK HEALTH REPORT] - Interval: {elapsed:.2f}s",
            border,
            f" Hardware Power / Throttling : {throttled_str}",
            f" CPU Temp / Load Average     : {temp_str} | Load: {load_str}",
            f" Wi-Fi RSSI / Signal Power   : {wifi_stats['rssi_dbm']} ({wifi_stats['signal_eval']})",
            f" Wi-Fi Link Quality / Noise  : Quality: {wifi_stats['link']} | Noise: {wifi_stats['noise_dbm']}",
            f" Wi-Fi Transmit Bitrate      : {wifi_stats['bitrate']}",
            f" Wi-Fi Power Management      : {wifi_stats['power_save']}",
            border,
            " [TOPIC MESSAGE RATES (Hz)]",
        ]

        # Categorize topic frequencies
        swarm_rates = []
        vehicle_rates = defaultdict(dict)

        for topic, count in snapshot.items():
            rate = count / elapsed if elapsed > 0 else 0.0
            if "/uav_" in topic:
                match = re.search(r'/uav_(\d+)/fmu/(in|out)/([a-zA-Z0-9_]+)', topic)
                if match:
                    uav_id, direction, name = match.groups()
                    vehicle_rates[f"Drone {uav_id}"][f"{direction}/{name}"] = rate
            else:
                swarm_rates.append((topic, rate))

        for topic, rate in sorted(swarm_rates):
            warn = " [LOW RATE!]" if rate < 1.0 and "live" in topic else ""
            lines.append(f"  * {topic:<35} : {rate:5.1f} Hz{warn}")

        for drone, topic_map in sorted(vehicle_rates.items()):
            lines.append(f"  --- {drone} ---")
            for name, rate in sorted(topic_map.items()):
                lines.append(f"    * {name:<33} : {rate:5.1f} Hz")

        lines.append(border)
        output_str = "\n".join(lines)

        # Escalate log level if hardware or Wi-Fi power saving flags are detected
        if throttled_val != 0 or "CRITICAL" in wifi_stats['power_save']:
            self.get_logger().error(output_str)
        else:
            self.get_logger().info(output_str)

def _read_wifi_link(self):
        """Read detailed RSSI, Link Quality, Bitrate, and Power Management status."""
        stats = {
            "link": "N/A",
            "rssi_dbm": "N/A",
            "noise_dbm": "N/A",
            "bitrate": "N/A",
            "power_save": "N/A",
            "signal_eval": "UNKNOWN",
        }
        
        # 1. Fast read via /proc/net/wireless (zero CPU overhead)
        try:
            if os.path.exists("/proc/net/wireless"):
                with open("/proc/net/wireless", "r") as f:
                    for line in f:
                        if self.wifi_interface in line:
                            parts = line.split()
                            # Quality link / level / noise
                            stats["link"] = parts[2].rstrip('.')
                            rssi_raw = float(parts[3].rstrip('.'))
                            stats["rssi_dbm"] = f"{rssi_raw:.0f} dBm"
                            stats["noise_dbm"] = f"{parts[4].rstrip('.')} dBm"
                            
                            # Evaluation scale for high-speed swarm telemetry
                            if rssi_raw >= -55:
                                stats["signal_eval"] = "EXCELLENT"
                            elif rssi_raw >= -68:
                                stats["signal_eval"] = "GOOD"
                            elif rssi_raw >= -78:
                                stats["signal_eval"] = "MARGINAL (Packet drops likely)"
                            else:
                                stats["signal_eval"] = "CRITICAL (Unstable link)"
        except Exception:
            pass

        # 2. Detailed link inspection via 'iw dev <iface> link' if /proc had missing fields
        try:
            res_link = subprocess.run(
                ['iw', 'dev', self.wifi_interface, 'link'],
                capture_output=True,
                text=True,
                timeout=0.2,
            )
            out = res_link.stdout
            
            # Extract signal dBm if not found in /proc
            if stats["rssi_dbm"] == "N/A":
                sig_match = re.search(r'signal:\s*(-?\d+)\s*dBm', out)
                if sig_match:
                    val = float(sig_match.group(1))
                    stats["rssi_dbm"] = f"{val:.0f} dBm"
                    stats["signal_eval"] = (
                        "EXCELLENT" if val >= -55 else
                        "GOOD" if val >= -68 else
                        "MARGINAL" if val >= -78 else "CRITICAL"
                    )

            # Extract TX Bitrate (e.g. 54.0 MBit/s)
            bitrate_match = re.search(r'tx bitrate:\s*([0-9.]+\s*MBit/s)', out)
            if bitrate_match:
                stats["bitrate"] = bitrate_match.group(1)

        except Exception:
            pass

        # 3. Power management status
        try:
            res_ps = subprocess.run(
                ['iw', 'dev', self.wifi_interface, 'get', 'power_save'],
                capture_output=True,
                text=True,
                timeout=0.2,
            )
            if "Power save: on" in res_ps.stdout:
                stats["power_save"] = "CRITICAL: ON (Enters sleep; drops UDP DDS packets)"
            elif "Power save: off" in res_ps.stdout:
                stats["power_save"] = "OFF (Optimal low-latency)"
        except Exception:
            pass

        return stats
def main(args=None):
    rclpy.init(args=args)
    node = SwarmLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()