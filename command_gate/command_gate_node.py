#!/usr/bin/env python3
"""
command_gate_node.py — Python implementation of the CommandGate safety relay.

Gate logic: gate_open = (heartbeat_ok OR !require_heartbeat) AND (enabled OR !require_enable)

NOTE: Assumes single-threaded executor. Loading into a MultiThreadedExecutor would require
mutex protection on heartbeat_ok_, enabled_, and channel last_fallback_bytes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


@dataclass
class Channel:
    input_topic: str
    output_topic: str
    message_type: str
    fallback_topic: str
    msg_class: object = None
    input_sub: object = None
    output_pub: object = None
    fallback_sub: object = None
    last_fallback_msg: object = None
    zero_msg: object = None
    warned_drop: bool = False


class CommandGateNode(Node):
    def __init__(self):
        super().__init__('command_gate')
        self._declare_and_get_params()
        self._setup_channels()
        self._setup_heartbeat()
        self._setup_enable()

        timer_period = self._heartbeat_timeout / 4.0
        self._heartbeat_timer = self.create_timer(timer_period, self._heartbeat_check_cb)

        self.get_logger().info(
            f'CommandGateNode started: {len(self._channels)} channel(s), '
            f'require_heartbeat={self._require_heartbeat}, '
            f'require_enable={self._require_enable}'
        )

    # ------------------------------------------------------------------ params

    def _declare_and_get_params(self) -> None:
        self.declare_parameter('num_channels', 1)
        self.declare_parameter('require_heartbeat', True)
        self.declare_parameter('heartbeat_timeout', 1.0)
        self.declare_parameter('heartbeat_topic', '~/heartbeat')
        self.declare_parameter('require_enable', False)
        self.declare_parameter('enable_topic', '~/enable')

        self._num_channels: int = self.get_parameter('num_channels').value
        self._require_heartbeat: bool = self.get_parameter('require_heartbeat').value
        self._heartbeat_timeout: float = self.get_parameter('heartbeat_timeout').value
        self._heartbeat_topic: str = self.get_parameter('heartbeat_topic').value
        self._require_enable: bool = self.get_parameter('require_enable').value
        self._enable_topic: str = self.get_parameter('enable_topic').value

        # Gate state
        self._enabled: bool = False
        self._heartbeat_ok: bool = False
        # Initialize as timed-out so gate starts closed when require_heartbeat=True
        self._last_heartbeat_time = (
            self.get_clock().now().nanoseconds * 1e-9
            - self._heartbeat_timeout - 1.0
        )

    # ---------------------------------------------------------------- channels

    def _setup_channels(self) -> None:
        self._channels: list[Channel] = []

        for i in range(self._num_channels):
            prefix = f'channel_{i}'
            self.declare_parameter(f'{prefix}.input_topic', '')
            self.declare_parameter(f'{prefix}.output_topic', '')
            self.declare_parameter(f'{prefix}.message_type', '')
            self.declare_parameter(f'{prefix}.fallback_topic', '')

            ch = Channel(
                input_topic=self.get_parameter(f'{prefix}.input_topic').value,
                output_topic=self.get_parameter(f'{prefix}.output_topic').value,
                message_type=self.get_parameter(f'{prefix}.message_type').value,
                fallback_topic=self.get_parameter(f'{prefix}.fallback_topic').value,
            )

            if not ch.input_topic or not ch.output_topic or not ch.message_type:
                self.get_logger().warn(
                    f'Channel {i} missing required parameters, skipping.'
                )
                continue

            try:
                ch.msg_class = get_message(ch.message_type)
            except Exception as exc:
                self.get_logger().warn(
                    f'Channel {i}: cannot resolve message type '
                    f'"{ch.message_type}": {exc}, skipping.'
                )
                continue

            ch.output_pub = self.create_publisher(ch.msg_class, ch.output_topic, 10)

            ch.input_sub = self.create_subscription(
                ch.msg_class, ch.input_topic,
                lambda msg, c=ch: self._on_input(msg, c), 10
            )

            if ch.fallback_topic:
                ch.fallback_sub = self.create_subscription(
                    ch.msg_class, ch.fallback_topic,
                    lambda msg, c=ch: self._on_fallback(msg, c), 1
                )

            try:
                ch.zero_msg = ch.msg_class()
            except Exception as exc:
                self.get_logger().warn(
                    f'Channel {i}: cannot build zero message for '
                    f'"{ch.message_type}": {exc}'
                )

            self._channels.append(ch)

    # -------------------------------------------------------- heartbeat / enable

    def _setup_heartbeat(self) -> None:
        from std_msgs.msg import Empty  # local import keeps top-level clean
        self._heartbeat_sub = self.create_subscription(
            Empty, self._heartbeat_topic, self._on_heartbeat_cb, 10
        )

    def _setup_enable(self) -> None:
        self._enable_sub = self.create_subscription(
            Bool, self._enable_topic, self._on_enable_topic_cb, 10
        )
        self._set_enabled_srv = self.create_service(
            SetBool, '~/set_enabled', self._on_set_enabled_cb
        )

    # ---------------------------------------------------------------- callbacks

    def _on_heartbeat_cb(self, _msg) -> None:
        was_open = self._is_gate_open()
        self._last_heartbeat_time = self.get_clock().now().nanoseconds * 1e-9
        self._heartbeat_ok = True
        now_open = self._is_gate_open()
        if not was_open and now_open:
            self._on_gate_state_change(False, True, self._open_reason('heartbeat received'))

    def _heartbeat_check_cb(self) -> None:
        was_open = self._is_gate_open()
        if self._require_heartbeat:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            elapsed = now_sec - self._last_heartbeat_time
            prev_ok = self._heartbeat_ok
            self._heartbeat_ok = elapsed < self._heartbeat_timeout
            now_open = self._is_gate_open()
            if was_open and not now_open:
                self._on_gate_state_change(True, False, 'heartbeat timeout')
            elif not prev_ok and self._heartbeat_ok and not was_open and now_open:
                self._on_gate_state_change(False, True, self._open_reason('heartbeat received'))

    def _on_enable_topic_cb(self, msg: Bool) -> None:
        was_open = self._is_gate_open()
        self._enabled = msg.data
        now_open = self._is_gate_open()
        if was_open != now_open:
            reason = self._open_reason('enabled via topic') if now_open else 'disabled via topic'
            self._on_gate_state_change(was_open, now_open, reason)

    def _on_set_enabled_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        was_open = self._is_gate_open()
        self._enabled = request.data
        now_open = self._is_gate_open()
        if was_open != now_open:
            reason = self._open_reason('enabled via service') if now_open else 'disabled via service'
            self._on_gate_state_change(was_open, now_open, reason)
        response.success = True
        response.message = f'enabled set to {self._enabled}'
        return response

    def _on_input(self, msg, ch: Channel) -> None:
        if self._is_gate_open():
            ch.output_pub.publish(msg)

    def _on_fallback(self, msg, ch: Channel) -> None:
        ch.last_fallback_msg = msg

    # ---------------------------------------------------------------- gate logic

    def _is_gate_open(self) -> bool:
        hb_ok = self._heartbeat_ok or not self._require_heartbeat
        en_ok = self._enabled or not self._require_enable
        return hb_ok and en_ok

    def _open_reason(self, trigger: str) -> str:
        if self._require_heartbeat and self._require_enable:
            if 'heartbeat' in trigger:
                return 'heartbeat received and already enabled'
            return f'heartbeat alive and {trigger}'
        return trigger

    def _on_gate_state_change(self, was_open: bool, now_open: bool, reason: str) -> None:
        if now_open:
            self.get_logger().info(f'Gate OPEN — {reason}')
            for ch in self._channels:
                ch.warned_drop = False
        else:
            self.get_logger().info(f'Gate CLOSED — {reason}')
            for ch in self._channels:
                self._publish_fallback_or_zero(ch)

    def _publish_fallback_or_zero(self, ch: Channel) -> None:
        if ch.last_fallback_msg is not None:
            ch.output_pub.publish(ch.last_fallback_msg)
        elif ch.zero_msg is not None:
            ch.output_pub.publish(ch.zero_msg)
        elif not ch.warned_drop:
            self.get_logger().warn(
                f'Gate closed on channel [{ch.output_topic}]: '
                f'no fallback or zero message available, dropping.'
            )
            ch.warned_drop = True


def main(args=None):
    rclpy.init(args=args)
    node = CommandGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
