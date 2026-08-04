# Kortex API servoing mode constants
# Reference: Kinova-kortex2_Gen3_G3L/linked_md/python_servoing_modes.md
# SINGLE_LEVEL_SERVOING = 1   # Default! high-level control at 40 Hz (re kinova manual)
# ADMITTANCE_MODE       = 2   # Per-joint gravity compensation
# LOW_LEVEL_SERVOING    = 3   # Direct actuator commands at 1 kHz (C++ ONLY!) DO NOT USE.

import os

FAKE_HARDWARE = os.environ.get('FAKE_HARDWARE', '0').lower() in ('1', 'true', 'yes')

# Default credentials for Kinova Gen3
ROBOT_IP = os.environ.get('ROBOT_IP', '192.168.0.10')  # default NOT .1.10
ROBOT_PORT = 10000
USERNAME = 'admin'
PASSWORD = 'admin'

if not FAKE_HARDWARE:
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.messages import Base_pb2, Session_pb2
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport

    # Kortex API servoing mode enum values
    SINGLE_LEVEL_SERVOING = Base_pb2.SINGLE_LEVEL_SERVOING
    ADMITTANCE = Base_pb2.JOINT

# from kortex_driver.srv import SetServoingMode, SetAdmittance
# from kortex_driver.msg import AdmittanceModeInformation

class CompliantModeManager:
    """
    Connects directly to the Gen3 via gRPC and toggles admittance mode.
    Call enable() before hand-guiding, disable() before any trajectory.

    Set FAKE_HARDWARE=1 in the environment to run without a physical robot;
    all gRPC calls become no-ops and are logged to stdout.
    """

    def __init__(self, ip=ROBOT_IP):
        if FAKE_HARDWARE:
            print('[FAKE] CompliantModeManager: skipping robot gRPC connection.')
            return
        self._transport = TCPTransport()
        self._transport.connect(ip, ROBOT_PORT)
        self._router  = RouterClient(self._transport, lambda x: None)
        self._session = SessionManager(self._router)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username       = USERNAME
        session_info.password       = PASSWORD
        session_info.session_inactivity_timeout   = 60000
        session_info.connection_inactivity_timeout = 2000
        self._session.CreateSession(session_info)

        self._base = BaseClient(self._router)
        print('Connected to Gen3 via Kortex API.')

    def enable(self):
        """Switch all joints to admittance (gravity-compensated) mode."""
        if FAKE_HARDWARE:
            print('[FAKE] Admittance mode ON. move the arm by hand (simulated).')
            return
        mode = Base_pb2.ServoingModeInformation()
        mode.servoing_mode = SINGLE_LEVEL_SERVOING
        self._base.SetServoingMode(mode)

        admittance = Base_pb2.Admittance()
        admittance.admittance_mode = ADMITTANCE
        self._base.SetAdmittance(admittance)
        print('Admittance mode ON. you can now move the arm by hand.')

    def disable(self):
        """Return to normal position control."""
        if FAKE_HARDWARE:
            print('[FAKE] Admittance mode OFF. position control restored (simulated).')
            return
        mode = Base_pb2.ServoingModeInformation()
        mode.servoing_mode = SINGLE_LEVEL_SERVOING
        self._base.SetServoingMode(mode)

        admittance = Base_pb2.Admittance()
        admittance.admittance_mode = Base_pb2.DISABLED
        self._base.SetAdmittance(admittance)
        print('Admittance mode OFF. position control restored.')

    def go_home(self):
        """Execute the built-in 'Home' named position (arm folds to safe pose)."""
        if FAKE_HARDWARE:
            print('[FAKE] go_home: arm would return to Home pose.')
            return
        # Scan all stored actions. Home is a REACH_JOINT_ANGLES
        # action internally, there is no REACH_NAMED_POSITION enum value.
        action_handle = None
        for action in self._base.ReadAllActions(Base_pb2.RequestedActionType()).action_list:
            if action.name == 'Home':
                action_handle = action.handle
                break
        if action_handle is None:
            print('go_home: could not find "Home" named position — skipping.')
            return
        self._base.ExecuteActionFromReference(action_handle)
        print('go_home: arm returning to Home position.')

    def close(self):
        if FAKE_HARDWARE:
            return
        self._session.CloseSession()
        self._transport.disconnect()
