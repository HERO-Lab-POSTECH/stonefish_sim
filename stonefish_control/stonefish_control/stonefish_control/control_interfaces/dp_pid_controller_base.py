# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ROS2 Port: Copyright 2025

import numpy as np
from stonefish_control_msgs.srv import SetPIDParams, GetPIDParams
from .dp_controller_base import DPControllerBase
from rcl_interfaces.msg import SetParametersResult


class DPPIDControllerBase(DPControllerBase):
    """Abstract class for PID-based controllers. The base
    class method `update_controller` must be overridden
    in other for a controller to work.
    """

    def __init__(self, node_name, *args):
        # Start the super class
        DPControllerBase.__init__(self, node_name, *args)

        # Proportional gains
        self._Kp = np.zeros(shape=(6, 6))
        # Derivative gains
        self._Kd = np.zeros(shape=(6, 6))
        # Integral gains
        self._Ki = np.zeros(shape=(6, 6))
        # Integrator component
        self._int = np.zeros(6)
        # Error for the vehicle pose
        self._error_pose = np.zeros(6)
        # Filtered velocity error (for derivative term)
        self._filtered_vel_error = np.zeros(6)

        # Flag to suppress duplicate parameter logs (when set via service)
        self._suppress_param_log = False

        # Declare and get PID parameters (array format)
        self.declare_parameter('Kp', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('Kd', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('Ki', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Load initial values from array parameters
        Kp_diag = self.get_parameter('Kp').value
        Kd_diag = self.get_parameter('Kd').value
        Ki_diag = self.get_parameter('Ki').value

        if len(Kp_diag) == 6 and len(Kd_diag) == 6 and len(Ki_diag) == 6:
            self._Kp = np.diag(Kp_diag)
            self._Kd = np.diag(Kd_diag)
            self._Ki = np.diag(Ki_diag)
        else:
            raise ValueError('PID matrix error: 6 coefficients needed')

        # Derivative filter time constant (low-pass filter to reduce noise)
        # Increased from 0.05 to 0.15 for better noise rejection
        self.declare_parameter('derivative_filter_tau', 0.15)
        self._derivative_filter_tau = self.get_parameter('derivative_filter_tau').value

        # Create services
        self._set_pid_params_service = self.create_service(
            SetPIDParams,
            'set_pid_params',
            self.set_pid_params_callback)

        self._get_pid_params_service = self.create_service(
            GetPIDParams,
            'get_pid_params',
            self.get_pid_params_callback)

        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self._on_parameter_event)

        self.get_logger().info('✓ PID controller ready')

    def _reset_controller(self):
        """Reset reference and and error vectors."""
        super(DPPIDControllerBase, self)._reset_controller()
        self._error_pose = np.zeros(6)
        self._int = np.zeros(6)
        self._filtered_vel_error = np.zeros(6)

    def set_pid_params_callback(self, request, response):
        """Service callback function to set the PID's parameters.

        Also updates ROS parameters to keep service and param in sync.
        """
        kp = list(request.kp)
        kd = list(request.kd)
        ki = list(request.ki)

        if len(kp) != 6 or len(kd) != 6 or len(ki) != 6:
            response.success = False
            return response

        self._Kp = np.diag(kp)
        self._Ki = np.diag(ki)
        self._Kd = np.diag(kd)

        # CRITICAL: Reset integrator and filter states when gains change!
        # Otherwise, accumulated states from previous gains corrupt new gain evaluation
        self._int = np.zeros(6)
        self._error_pose = np.zeros(6)
        self._filtered_vel_error = np.zeros(6)

        # Sync with ROS parameters (bidirectional sync)
        # Set flag to suppress duplicate logs from parameter callback
        self._suppress_param_log = True
        from rclpy.parameter import Parameter
        self.set_parameters([
            Parameter('Kp', Parameter.Type.DOUBLE_ARRAY, kp),
            Parameter('Kd', Parameter.Type.DOUBLE_ARRAY, kd),
            Parameter('Ki', Parameter.Type.DOUBLE_ARRAY, ki)
        ])
        self._suppress_param_log = False

        response.success = True
        return response

    def get_pid_params_callback(self, request, response):
        """Service callback function to return the PID's parameters"""
        response.kp = [self._Kp[i, i] for i in range(6)]
        response.kd = [self._Kd[i, i] for i in range(6)]
        response.ki = [self._Ki[i, i] for i in range(6)]
        return response

    def _on_parameter_event(self, params):
        """Callback for parameter changes (dynamic reconfigure).

        Allows runtime adjustment of PID gains via ros2 param set.
        Example:
            ros2 param set /bluerov2/pid_controller Kp "[100.0, 100.0, 500.0, 50.0, 14.7, 4.5]"
        """
        result = SetParametersResult(successful=True)
        reset_states = False

        for param in params:
            if param.name == 'Kp':
                kp = param.value
                if len(kp) == 6:
                    self._Kp = np.diag(kp)
                    reset_states = True
                    # Only log if not suppressed (to avoid duplicate logs from service calls)
                    if not self._suppress_param_log:
                        self.get_logger().info(f'Kp updated to: {kp}')
                else:
                    result.successful = False
                    result.reason = 'Kp must have 6 values'

            elif param.name == 'Kd':
                kd = param.value
                if len(kd) == 6:
                    self._Kd = np.diag(kd)
                    reset_states = True
                    # Only log if not suppressed
                    if not self._suppress_param_log:
                        self.get_logger().info(f'Kd updated to: {kd}')
                else:
                    result.successful = False
                    result.reason = 'Kd must have 6 values'

            elif param.name == 'Ki':
                ki = param.value
                if len(ki) == 6:
                    self._Ki = np.diag(ki)
                    reset_states = True
                    # Only log if not suppressed
                    if not self._suppress_param_log:
                        self.get_logger().info(f'Ki updated to: {ki}')
                else:
                    result.successful = False
                    result.reason = 'Ki must have 6 values'

        # Reset states if any gain changed
        if reset_states:
            self._int = np.zeros(6)
            self._error_pose = np.zeros(6)
            self._filtered_vel_error = np.zeros(6)

        return result

    def update_pid(self):
        """Return the control signal computed from the PID algorithm.
        To implement a PID-based controller that inherits this class,
        call this function in the derived class' `update` method to
        obtain the control vector.

        Returns:
            numpy.array: Control signal
        """
        if not self.odom_is_init:
            return np.zeros(6)

        # Store current pose error
        prev_error = self._error_pose
        self._error_pose = self.error_pose_euler

        # Apply low-pass filter to velocity error (derivative term)
        # This reduces high-frequency noise amplification
        if self._derivative_filter_tau > 0 and self._dt > 0:
            # Apply exponential moving average filter
            alpha = self._dt / (self._dt + self._derivative_filter_tau)
            self._filtered_vel_error = alpha * self._errors['vel'] + (1 - alpha) * self._filtered_vel_error
        else:
            # No filtering (tau=0 or first iteration)
            self._filtered_vel_error = self._errors['vel']

        # Compute PID terms
        p_term = np.dot(self._Kp, self.error_pose_euler)
        d_term = np.dot(self._Kd, self._filtered_vel_error)
        i_term = np.dot(self._Ki, self._int)

        # Compute unsaturated control output
        tau_unsat = p_term + d_term + i_term

        # Apply saturation
        tau_sat = np.clip(tau_unsat, -self._control_saturation, self._control_saturation)

        # Anti-windup: Conditional integration
        # Only integrate when:
        # 1. Output is not saturated, OR
        # 2. Error sign opposes integrator sign (error is reducing)
        is_saturated = np.abs(tau_sat - tau_unsat) > 1e-6  # Check if clipping occurred
        for i in range(6):
            if not is_saturated[i] or np.sign(self.error_pose_euler[i]) != np.sign(self._int[i]):
                # Update integrator (trapezoidal integration)
                self._int[i] += 0.5 * (self.error_pose_euler[i] + prev_error[i]) * self._dt

        return tau_sat
