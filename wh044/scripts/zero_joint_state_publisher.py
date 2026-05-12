#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _parse_movable_joint_names(urdf_path: str) -> list[str]:
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError as e:
        raise RuntimeError(f'Failed to parse URDF XML: {urdf_path}: {e}')

    root = tree.getroot()
    joint_names: list[str] = []
    for joint in root.findall('joint'):
        joint_type = (joint.get('type') or '').strip().lower()
        if joint_type == 'fixed':
            continue
        name = (joint.get('name') or '').strip()
        if name:
            joint_names.append(name)

    # Keep deterministic order and avoid duplicates
    seen: set[str] = set()
    ordered: list[str] = []
    for n in joint_names:
        if n not in seen:
            seen.add(n)
            ordered.append(n)
    return ordered


class ZeroJointStatePublisher(Node):
    def __init__(self, urdf_path: str, rate_hz: float, topic: str):
        super().__init__('zero_joint_state_publisher')
        self._joint_names = _parse_movable_joint_names(urdf_path)
        if not self._joint_names:
            self.get_logger().warning('No movable joints found in URDF; publishing empty JointState')

        topic = (topic or 'joint_states').strip()
        if topic == '':
            topic = 'joint_states'
        self._pub = self.create_publisher(JointState, topic, 10)
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [0.0] * len(self._joint_names)
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Publish zero /joint_states based on a URDF file')
    parser.add_argument('--urdf', required=True, help='Path to a URDF file')
    parser.add_argument('--rate', type=float, default=30.0, help='Publish rate (Hz)')
    parser.add_argument('--topic', default='joint_states', help='JointState topic name')
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = ZeroJointStatePublisher(args.urdf, args.rate, args.topic)
        rclpy.spin(node)
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
