#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from stonefish_control_msgs.msg import TrajectoryPoint
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from stonefish_control.controllers.hybrid_controller import HybridController
from stonefish_control.control_interfaces import DynamicsLoader

class HybridController4DOFNode(Node):
    def __init__(self):
        super().__init__('hybrid_controller_4dof')
        self.dynamics = DynamicsLoader(self)
        self._setup_parameters()
        self._load_parameters()
        
        self.controller = HybridController(
            Kp_vel=self.Kp_vel, Kd_vel=self.Kd_vel, Ki_vel=self.Ki_vel, Kb_vel=self.Kb_vel,
            Kp_pos=self.Kp_pos, Kd_pos=self.Kd_pos, Ki_pos=self.Ki_pos, Kb_pos=self.Kb_pos,
            mass=self.dynamics.mass, inertia_zz=self.dynamics.inertia_zz,
            max_force_vel=self.max_force_vel, max_torque_vel=self.max_torque_vel,
            max_force_pos=self.max_force_pos, max_torque_pos=self.max_torque_pos,
            integral_safety_factor_vel=self.integral_safety_factor_vel,
            integral_safety_factor_pos=self.integral_safety_factor_pos,
            initial_mode=self.initial_mode
        )
        
        self.odom_received = False
        self.cmd_received = False
        self.current_pose = np.zeros(6)
        self.current_vel = np.zeros(6)
        self.desired_pose = np.zeros(4)
        self.desired_vel = None
        self.last_time = None
        self.control_dt = 1.0 / self.control_rate
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.odom_sub = self.create_subscription(Odometry, 'odometry', self.odom_callback, qos)
        self.cmd_sub = self.create_subscription(TrajectoryPoint, 'cmd_pose', self.cmd_callback, 10)
        self.mode_sub = self.create_subscription(String, 'control_mode', self.mode_callback, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'thruster_manager/input_stamped', 10)
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)
        self.log_timer = self.create_timer(2.0, self.log_status)
        
        self.get_logger().info(f'Hybrid Controller initialized (mode: {self.controller.control_mode})')
    
    def _setup_parameters(self):
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('initial_mode', 'velocity')
        self.declare_parameter('velocity_mode.Kp', [200.0, 200.0, 250.0, 150.0])
        self.declare_parameter('velocity_mode.Kd', [0.0, 100.0, 120.0, 80.0])
        self.declare_parameter('velocity_mode.Ki', [50.0, 50.0, 60.0, 10.0])
        self.declare_parameter('velocity_mode.Kb', [0.8, 0.8, 0.8, 0.8])
        self.declare_parameter('velocity_mode.max_force', 800.0)
        self.declare_parameter('velocity_mode.max_torque', 160.0)
        self.declare_parameter('velocity_mode.integral_safety_factor', 0.5)
        self.declare_parameter('position_mode.Kp', [300.0, 300.0, 400.0, 200.0])
        self.declare_parameter('position_mode.Kd', [150.0, 150.0, 200.0, 100.0])
        self.declare_parameter('position_mode.Ki', [10.0, 10.0, 20.0, 5.0])
        self.declare_parameter('position_mode.Kb', [0.8, 0.8, 0.8, 0.8])
        self.declare_parameter('position_mode.max_force', 200.0)
        self.declare_parameter('position_mode.max_torque', 50.0)
        self.declare_parameter('position_mode.integral_safety_factor', 2.0)
    
    def _load_parameters(self):
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.initial_mode = self.get_parameter('initial_mode').value
        self.Kp_vel = np.array(self.get_parameter('velocity_mode.Kp').value)
        self.Kd_vel = np.array(self.get_parameter('velocity_mode.Kd').value)
        self.Ki_vel = np.array(self.get_parameter('velocity_mode.Ki').value)
        self.Kb_vel = np.array(self.get_parameter('velocity_mode.Kb').value)
        self.max_force_vel = self.get_parameter('velocity_mode.max_force').value
        self.max_torque_vel = self.get_parameter('velocity_mode.max_torque').value
        self.integral_safety_factor_vel = self.get_parameter('velocity_mode.integral_safety_factor').value
        self.Kp_pos = np.array(self.get_parameter('position_mode.Kp').value)
        self.Kd_pos = np.array(self.get_parameter('position_mode.Kd').value)
        self.Ki_pos = np.array(self.get_parameter('position_mode.Ki').value)
        self.Kb_pos = np.array(self.get_parameter('position_mode.Kb').value)
        self.max_force_pos = self.get_parameter('position_mode.max_force').value
        self.max_torque_pos = self.get_parameter('position_mode.max_torque').value
        self.integral_safety_factor_pos = self.get_parameter('position_mode.integral_safety_factor').value
    
    def mode_callback(self, msg: String):
        mode = msg.data.lower()
        if mode in ['velocity', 'position']:
            old_mode = self.controller.control_mode
            self.controller.set_mode(mode)
            if mode != old_mode:
                self.get_logger().info(f'Control mode switched: {old_mode} → {mode}')
    
    def odom_callback(self, msg: Odometry):
        self.current_pose[0:3] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        r = Rotation.from_quat(quat)
        self.current_pose[3:6] = r.as_euler('xyz', degrees=False)
        self.current_vel[0:3] = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.current_vel[3:6] = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        self.odom_received = True
    
    def cmd_callback(self, msg: TrajectoryPoint):
        self.desired_pose[0:3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        r = Rotation.from_quat(quat)
        _, _, yaw = r.as_euler('xyz', degrees=False)
        self.desired_pose[3] = yaw
        self.desired_vel = np.array([msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z, msg.velocity.angular.z])
        self.cmd_received = True
    
    def control_loop(self):
        if not self.odom_received or not self.cmd_received:
            return
        current_time = self.get_clock().now()
        if self.last_time is None:
            dt = self.control_dt
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(0.001, min(dt, 0.1))
        self.last_time = current_time
        
        tau_6dof, _ = self.controller.compute_control(self.desired_pose, self.current_pose, self.current_vel, dt, self.desired_vel)
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = current_time.to_msg()
        wrench_msg.header.frame_id = f'{self.vehicle_name}/base_link'
        wrench_msg.wrench.force.x = tau_6dof[0]
        wrench_msg.wrench.force.y = tau_6dof[1]
        wrench_msg.wrench.force.z = tau_6dof[2]
        wrench_msg.wrench.torque.z = tau_6dof[5]
        self.wrench_pub.publish(wrench_msg)
    
    def log_status(self):
        if not self.odom_received or not self.cmd_received:
            return
        status = self.controller.get_status()
        self.get_logger().info(f"Mode: {status['mode']} | Switches: {status['switches']}")

def main(args=None):
    rclpy.init(args=args)
    node = HybridController4DOFNode()
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
