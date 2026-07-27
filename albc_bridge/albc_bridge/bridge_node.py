#!/usr/bin/env python3
"""ALBC obs->policy->action bridge node, closing the loop at 50Hz.

Stonefish sensors (/albc/odometry, /albc/joint_states) -> ObsBuilder (69D) ->
9-frame rolling window -> StudentTCN latent -> TeacherActor action (8D) ->
/albc/servos (joint targets) + /albc/setpoint/pwm (thruster PWM, ESC order).

=== Window construction (Step 0 SSOT) ===
StudentTCN.forward consumes (1, 9, 69) -- confirmed by golden_tcn.npz
(input_window.shape == (1, 9, 69), albc_bridge/policy/golden/golden_tcn.npz).
TCN_HISTORY=9 and the rolling-window construction are confirmed from the
deploy reference module in the marinelab-isaaclab container:
  constrained-albc/deploy/student_albc_260607/student_inference.py:80
      TCN_HISTORY = 9  # config.py:40 tcn_history=9 is the real trained value
  student_inference.py:282-287 (DeployedStudentPolicy.act, encoder_type=="tcn"):
      self._tcn_window.append(obs87)                       # newest -> rightmost
      while len(self._tcn_window) < TCN_HISTORY:
          self._tcn_window.appendleft(self._tcn_window[0])  # pad left w/ oldest avail.
      win = np.stack(self._tcn_window)[None]                # (1, 9, dim)
pack_B's npforward.py (which this node imports) states in its own docstring
(npforward.py:3) that it "Mirrors student_inference.py's torch modules
exactly", so this window contract carries over unchanged (obs dim 87->69 is
the only difference, per ObsBuilder's 69D deployed-contract note). No separate
board ROS node was reachable from this environment (hero_agent host clone has
no python inference node -- board runtime lives only on the physical
agent-jetson, out of reach here), so student_inference.py is the SSOT.

  UPDATE (2026-07-27, H5b fix): student_inference.py is self-contradictory
  on normalization; the canonical reference is
  constrained-albc/analysis/student_policy.py. The window passed to
  StudentTCN.forward MUST be teacher-normalized -- the student is distilled
  on normalized input and carries no normalizer of its own
  (npforward.py:139-144). See teacher.normalize() applied at the forward call.
Reproduced below with a plain deque(maxlen=9): append the current obs each
tick (rightmost = newest); on startup, pad left by duplicating the single
available frame until the window is full.
"""
import os
import sys
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from albc_bridge.obs_builder import ObsBuilder

_POLICY_DIR = os.path.join(get_package_share_directory('albc_bridge'), 'policy')
sys.path.insert(0, _POLICY_DIR)
from npforward import StudentTCN, TeacherActor  # noqa: E402

TCN_HISTORY = 9
CONTROL_DT = 1.0 / 50.0
JOINT_DELTA_SCALE = 0.10  # matches ObsBuilder docstring: q_des += delta_scale*action[:2]


class BridgeNode(Node):
    def __init__(self):
        super().__init__('albc_bridge')

        w_tcn = np.load(os.path.join(_POLICY_DIR, 'weights_tcn.npz'))
        w_teacher = np.load(os.path.join(_POLICY_DIR, 'weights_teacher.npz'))
        self.tcn = StudentTCN(w_tcn)
        self.teacher = TeacherActor(w_teacher)

        self.obs_builder = ObsBuilder()
        self.window = deque(maxlen=TCN_HISTORY)

        self.joint_targets = np.array([0.0, 1.5708])
        self.last_action = np.zeros(8)

        self.declare_parameter('ang_cmd', [0.0, 0.0, 0.0])

        self.odom = None
        self.joint_state = None

        self.create_subscription(Odometry, '/albc/odometry', self._on_odom, 10)
        self.create_subscription(JointState, '/albc/joint_states', self._on_joint_state, 10)
        self.servo_pub = self.create_publisher(JointState, '/albc/servos', 10)
        self.pwm_pub = self.create_publisher(Float64MultiArray, '/albc/setpoint/pwm', 10)
        # diagnostic: publish the student TCN latent (caution #2 encoder-health check)
        self.latent_pub = self.create_publisher(Float64MultiArray, '/albc/debug/latent', 10)

        self.create_timer(CONTROL_DT, self.on_tick)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_joint_state(self, msg):
        self.joint_state = msg

    def on_tick(self):
        if self.odom is None or self.joint_state is None:
            return

        q = self.odom.pose.pose.orientation
        odom_quat = np.array([q.x, q.y, q.z, q.w])
        av = self.odom.twist.twist.angular
        odom_angvel = np.array([av.x, av.y, av.z])

        # look up joint1/joint2 by name suffix -- Stonefish prefixes with the
        # vehicle name (e.g. 'albc/joint1'); Task-7 synthetic input used bare
        # 'joint1'. Match both, never assume array order.
        names = list(self.joint_state.name)

        def _find(suffix):
            for i, n in enumerate(names):
                if n == suffix or n.endswith('/' + suffix):
                    return i
            return None

        i1, i2 = _find('joint1'), _find('joint2')
        if i1 is None or i2 is None:
            self.get_logger().warn(f'joint_states missing joint1/joint2 (got {names}), skipping tick')
            return
        joint_pos = np.array([self.joint_state.position[i1], self.joint_state.position[i2]])
        joint_vel = np.array([self.joint_state.velocity[i1], self.joint_state.velocity[i2]])

        ang_cmd = np.array(self.get_parameter('ang_cmd').value)

        # 1. obs uses PREVIOUS joint_targets == q_des_{t-1} (Task 6 prev-timing contract)
        obs = self.obs_builder.update(
            odom_quat, odom_angvel, joint_pos, joint_vel,
            ang_cmd, self.joint_targets, self.last_action,
        )

        # rolling 9-frame window (see module docstring Step 0 SSOT)
        self.window.append(obs.astype(np.float32))
        while len(self.window) < TCN_HISTORY:
            self.window.appendleft(self.window[0])
        win = np.stack(self.window)[None]  # (1, 9, 72)

        # 2. policy forward
        latent = self.tcn.forward(self.teacher.normalize(win))
        lat_msg = Float64MultiArray()
        lat_msg.data = [float(v) for v in latent[0]]  # (9,) diagnostic
        self.latent_pub.publish(lat_msg)
        obs_norm = self.teacher.normalize(obs[None])
        action = self.teacher.act(obs_norm, latent)[0]  # (8,)

        # 3. NaN guard -- skip publish AND skip state updates
        if not np.all(np.isfinite(action)):
            self.get_logger().warn('non-finite action from policy, skipping publish')
            return

        # mirror Isaac _update_action_buffers/thruster.apply_dynamics: single clamp point
        action = np.clip(action, -1.0, 1.0)

        # 4. accumulate joint targets to q_des_t (only now, after obs/policy used prev)
        self.joint_targets = self.joint_targets + action[0:2] * JOINT_DELTA_SCALE

        # 5. publish
        servo_msg = JointState()
        servo_msg.name = [names[i1], names[i2]]  # echo Stonefish's exact joint names (e.g. albc/joint1)
        servo_msg.position = [float(v) for v in self.joint_targets]
        self.servo_pub.publish(servo_msg)

        pwm_msg = Float64MultiArray()
        pwm_msg.data = [float(v) for v in action[2:8]]  # ESC declaration order, unmodified
        self.pwm_pub.publish(pwm_msg)

        # 6. carry action forward for next tick's obs/thruster-filter update
        self.last_action = action


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
