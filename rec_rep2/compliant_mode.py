# Kortex API servoing mode constants
# Reference: Kinova-kortex2_Gen3_G3L/linked_md/python_servoing_modes.md
SINGLE_LEVEL_SERVOING = 1   # Default! high-level control at 40 Hz (re kinova manual)
ADMITTANCE_MODE       = 2   # Per-joint gravity compensation
LOW_LEVEL_SERVOING    = 3   # Direct actuator commands at 1 kHz (C++ ONLY!) DO NOT USE.

import rclpy
from rclpy.node import Node
from kortex_driver.srv import SetServoingMode, SetAdmittanceMode
from kortex_driver.msg import AdmittanceModeInformation

class CompliantModeManager(Node):
    """
    Toggles Kinova Gen3 joints into admittance (compliant) mode
    using the Kortex API services exposed by ros2_kortex.

    Services used:
      /kortex_driver/base/set_servoing_mode
      /kortex_driver/base/set_admittance_mode
    """

    def __init__(self):
        super().__init__('compliant_mode_manager')
        self._servo_cli = self.create_client(
            SetServoingMode,
            '/kortex_driver/base/set_servoing_mode',
        )
        self._admit_cli = self.create_client(
            SetAdmittanceMode,
            '/kortex_driver/base/set_admittance_mode',
        )
        # Wait for services to become available
        self._servo_cli.wait_for_service(timeout_sec=5.0)
        self._admit_cli.wait_for_service(timeout_sec=5.0)

    async def enable(self):
        """Put all 7 joints into admittance (compliant) mode."""
        # Step 1: ensure high-level servoing is active
        req = SetServoingMode.Request()
        req.input.servoing_mode = SINGLE_LEVEL_SERVOING

        """Make sure services actually succeed before moving the arm"""
        future = SetServoingMode.Request()
        req.input.servoing_mode = SINGLE_LEVEL_SERVOING

        future = self._servo_cli.call_async(req)
        await future # wait for response.

        #self._servo_cli.call(req)

        # Step 2: set each joint to admittance control
        req2 = SetAdmittanceMode.Request()
        for joint_id in range(1, 8):           # joints 1-7
            info = AdmittanceModeInformation()
            info.joint_handle  = joint_id
            #info.joint_id = joint_id
            info.control_mode  = ADMITTANCE_MODE
            req2.input.joints.append(info)
        self._admit_cli.call(req2)

        self.get_logger().info(
            'All joints now in admittance mode. Gravity compensation active.'
            'You can move the arm by hand.'
        )

    def disable(self):
        """Return all joints to normal position control."""
        req = SetServoingMode.Request()
        req.input.servoing_mode = SINGLE_LEVEL_SERVOING
        self._servo_cli.call(req)
        self.get_logger().info('Returned to position control mode.')