import time
import requests
import threading
import math  # Needed for radians conversion

import pybullet as p
import pybullet_data
# import numpy as np
# import cv2  # Optional for displaying images

# # Connect to PyBullet
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# # Load your URDF model (update path if necessary)
# robot_id = p.loadURDF("F:\\PycharmProjects\\URDF_RobotArm\\my_robot_original\\urdf\\my_robot.urdf", useFixedBase=True)

# # Define camera properties
# width, height = 640, 480  # Image resolution
# fov = 60  # Field of View
# aspect = width / height
# near, far = 0.1, 10  # Clipping planes

# # Define camera positions & targets
# camera_configs = {
#     "Top View": {"pos": [0, 0, 3], "target": [0, 0, 0]},  # Directly above
#     "Side View": {"pos": [3, 0, 1], "target": [0, 0, 1]},  # Side view
#     "Angled View": {"pos": [2, 2, 1.5], "target": [0, 0, 0]}  # Angled perspective
# }

# # Function to capture images from all cameras
# def capture_images():
#     images = {}
    
#     for view_name, cam in camera_configs.items():
#         # Compute view and projection matrices
#         view_matrix = p.computeViewMatrix(cam["pos"], cam["target"], [0, 0, 1])
#         proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

#         # Capture camera image
#         img_arr = p.getCameraImage(width, height, view_matrix, proj_matrix)
#         rgb_array = np.reshape(img_arr[2], (height, width, 4))[:, :, :3]  # Remove alpha channel

#         images[view_name] = rgb_array

#         # Display image using OpenCV
#         cv2.imshow(view_name, rgb_array)

#     cv2.waitKey(1)  # Refresh OpenCV windows


# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For default assets

# Load the URDF model
robot_id = p.loadURDF("F:\\PycharmProjects\\URDF_RobotArm\\my_robot_original\\urdf\\my_robot.urdf", useFixedBase=True)

# Set up the camera
# p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
# p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=0, cameraTargetPosition=[0, 0, 0])
# p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-90, cameraPitch=0, cameraTargetPosition=[0, 0, 0])

# Set initial camera view to top view
camera_distance = .75  # Adjust this value to set the desired distance
p.resetDebugVisualizerCamera(
    cameraDistance=camera_distance,
    cameraYaw=0,
    cameraPitch=-89.9,
    cameraTargetPosition=[0, 0, 0]
)

# Variable to keep track of the current view
current_view = "top"

def toggle_view():
    global current_view
    if current_view == "top":
        # Switch to side view
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[0, 0, 0]
        )
        current_view = "side"
    else:
        # Switch to top view
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=0,
            cameraPitch=-89.9,
            cameraTargetPosition=[0, 0, 0]
        )
        current_view = "top"


# Debug visualizer settings
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

# Disable self-collision
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    for j in range(num_joints):
        p.setCollisionFilterPair(robot_id, robot_id, i, j, enableCollision=0)

# Print joint info and create a joint index mapping
joint_indices = {}
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_indices[joint_name] = i
    print(f"Joint {i}: {joint_name}")

FLASK_URL = "http://127.0.0.1:5000/return_positions"

# Function to fetch positions from Flask
def fetch_positions():
    while True:
        try:
            response = requests.get(FLASK_URL)
            if response.status_code == 200:
                raw_data = response.json()
                
                if isinstance(raw_data, str):
                    motor_positions = raw_data.replace(",", ".").split("&")
                elif isinstance(raw_data, list):
                    motor_positions = [str(val).replace(",", ".") for val in raw_data]
                else:
                    print(f"Unexpected data format: {raw_data}")
                    continue

                if len(motor_positions) == 14: 
                    round_value = 4
                    UpperRing_Rotation = round(float(motor_positions[0])+120+30, round_value)
                    MiddleRing_Rotation = round(float(motor_positions[1])+120+60, round_value) 
                    LowerRing_Rotation = round(float(motor_positions[2])+120, round_value) 

                    Base_Rotation = round(-float(motor_positions[3]), round_value)-180 # to accommodate the setup pose
                    LowerHinge_Rotation = round(-float(motor_positions[4]), round_value)+30 # to accommodate the setup pose
                    UpperHinge_Rotation = round(float(motor_positions[5]), round_value)-30 # to accommodate the setup pose
                    EndEffector_Rotation = round(-float(motor_positions[6]), round_value)
                    GripperState = round(float(motor_positions[7]), round_value)

                    UpperRing_Arm_Rotation = round(float(motor_positions[8]), round_value) 
                    MiddleRing_Arm_Rotation = round(-float(motor_positions[9]), round_value) 
                    LowerRing_Arm_Rotation = round(-float(motor_positions[10]), round_value)
                    
                    scale_factor_platform = 1
                    y_rotation = round(-float(motor_positions[11])*scale_factor_platform, round_value)
                    z_rotation = round(float(motor_positions[12])*scale_factor_platform, round_value)
                    x_rotation = round(-float(motor_positions[13])*scale_factor_platform, round_value)
                    
                    # Send data to simulation
                    update_simulation(
                        UpperRing_Rotation, MiddleRing_Rotation,
                        LowerRing_Rotation, Base_Rotation,
                        LowerHinge_Rotation, UpperHinge_Rotation,
                        EndEffector_Rotation, GripperState,
                        UpperRing_Arm_Rotation, MiddleRing_Arm_Rotation,
                        LowerRing_Arm_Rotation, x_rotation, y_rotation, z_rotation
                    )
                else:
                    print(f"Received unexpected number of values: {motor_positions}")

        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")

        time.sleep(0.1)  # Fetch positions every 100ms


# Function to update PyBullet with new positions
def update_simulation(
        
    UpperRing_Rotation, MiddleRing_Rotation,LowerRing_Rotation, 

    Base_Rotation,LowerHinge_Rotation, UpperHinge_Rotation,EndEffector_Rotation, 

    GripperState,

    UpperRing_Arm_Rotation, MiddleRing_Arm_Rotation,LowerRing_Arm_Rotation, 

    x_rotation, y_rotation, z_rotation):
    
    try:
        print(f"Received Positions:\n"
      f"Base_Rotation [3]: {Base_Rotation}, LowerHinge_Rotation [4]: {LowerHinge_Rotation}, UpperHinge_Rotation [5]: {UpperHinge_Rotation}, EndEffector_Rotation [6]: {EndEffector_Rotation}, GripperState: {GripperState}\n"
      f"UpperRing_Rotation [0]: {UpperRing_Rotation}, MiddleRing_Rotation [1]: {MiddleRing_Rotation}, LowerRing_Rotation [2]: {LowerRing_Rotation}\n"
      f"UpperRing_Arm_Rotation [8]: {UpperRing_Arm_Rotation}, MiddleRing_Arm_Rotation [7]: {MiddleRing_Arm_Rotation}, LowerRing_Arm_Rotation [9]: {LowerRing_Arm_Rotation}\n"
      f"Platform x_rotation [10]: {x_rotation}, y_rotation [11]: {y_rotation}, z_rotation [12]: {z_rotation}\n")
        # Convert degrees to radians
        values_in_radians = [
            math.radians(UpperRing_Rotation),
            math.radians(MiddleRing_Rotation),
            math.radians(LowerRing_Rotation),
            math.radians(Base_Rotation),
            math.radians(LowerHinge_Rotation),
            math.radians(UpperHinge_Rotation),
            math.radians(EndEffector_Rotation),
            math.radians(UpperRing_Arm_Rotation),
            math.radians(MiddleRing_Arm_Rotation),
            math.radians(LowerRing_Arm_Rotation),
            math.radians(x_rotation),
            math.radians(y_rotation),
            math.radians(z_rotation)
        ]

        # Define joint indices based on the setup
        # Joint 0: _RotationBase
        # Joint 1: _RotationHingeLower
        # Joint 2: _RotationHingeUpper
        # Joint 3: _RotationUpper
        # Joint 4: LongArm
        # Joint 5: LowerArmUpper
        # Joint 6: MiddleArm
        # Joint 7: MiddleArmUpper
        # Joint 8: ShortArm
        # Joint 9: ShortArmUpper
        # Joint 10: __Roll
        # Joint 11: __Pitch
        # Joint 12: __Yaw
        # Joint 13: __GripperLeft
        # Joint 14: __GripperRight

        joint_indices = {
            "Base_Rotation": 0, # Joint 0: _RotationBase
            "LowerHinge_Rotation": 1, # Joint 1: _RotationHingeLower
            "UpperHinge_Rotation": 2, # Joint 2: _RotationHingeUpper
            "EndEffector_Rotation": 3, # Joint 3: _RotationUpper
            
            "LowerRing_Rotation": 4, # Joint 4: LongArm
            "LowerRing_Arm_Rotation": 5, # Joint 5: LowerArmUpper

            "MiddleRing_Rotation": 6, # Joint 6: MiddleArm
            "MiddleRing_Arm_Rotation": 7, # Joint 7: MiddleArmUpper

            "UpperRing_Rotation": 8, # Joint 8: ShortArm
            "UpperRing_Arm_Rotation": 9, # Joint 9: ShortArmUpper

            "x_rotation": 10, # Joint 10: __Roll
            "y_rotation": 11, # Joint 11: __Pitch
            "z_rotation": 12, # Joint 12: __Yaw
            "GripperLeft": 13, # Joint 13: __GripperLeft
            "GripperRight": 14 # Joint 14: __GripperRight
        }

        # Apply values to PyBullet joints
        p.resetJointState(robot_id, joint_indices["Base_Rotation"], values_in_radians[3])
        p.resetJointState(robot_id, joint_indices["LowerHinge_Rotation"], values_in_radians[4])
        p.resetJointState(robot_id, joint_indices["UpperHinge_Rotation"], values_in_radians[5])
        p.resetJointState(robot_id, joint_indices["EndEffector_Rotation"], values_in_radians[6])

        p.resetJointState(robot_id, joint_indices["LowerRing_Rotation"], values_in_radians[2])
        p.resetJointState(robot_id, joint_indices["LowerRing_Arm_Rotation"], values_in_radians[9])

        p.resetJointState(robot_id, joint_indices["MiddleRing_Rotation"], values_in_radians[1])
        p.resetJointState(robot_id, joint_indices["MiddleRing_Arm_Rotation"], values_in_radians[8])

        p.resetJointState(robot_id, joint_indices["UpperRing_Rotation"], values_in_radians[0])
        p.resetJointState(robot_id, joint_indices["UpperRing_Arm_Rotation"], values_in_radians[7])

        p.resetJointState(robot_id, joint_indices["x_rotation"], values_in_radians[10])
        p.resetJointState(robot_id, joint_indices["y_rotation"], values_in_radians[11])
        p.resetJointState(robot_id, joint_indices["z_rotation"], values_in_radians[12])

        # Assuming GripperState affects both GripperLeft and GripperRight equally
        gripper_position = math.radians(GripperState)
        p.resetJointState(robot_id, joint_indices["GripperLeft"], gripper_position)
        p.resetJointState(robot_id, joint_indices["GripperRight"], gripper_position)

    except Exception as e:
        print(f"An error occurred: {e}")

# Start position fetching thread
threading.Thread(target=fetch_positions, daemon=True).start()

# Run PyBullet simulation
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240)

    # Check for keypress events
    keys = p.getKeyboardEvents()
    if ord(' ') in keys:  # Check if spacebar is pressed
        toggle_view()