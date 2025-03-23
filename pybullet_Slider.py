import time
import requests
import threading
import math
import pybullet as p
import pybullet_data

# Connect to PyBullet with GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# URDF path - update this to your path
URDF_PATH = "F:\\PycharmProjects\\URDF_RobotArm\\my_robot_original\\urdf\\my_robot.urdf"
FLASK_URL = "http://127.0.0.1:5000/return_positions"

# Load the robot with fixed base
robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)

# Disable self-collision for all joints
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    for j in range(num_joints):
        p.setCollisionFilterPair(robot_id, robot_id, i, j, enableCollision=0)

# UI Settings
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Enable GUI for debugging
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

# Create debugging sliders for each joint
sliders = {}

# Print joint info and create debug parameter sliders
print("\n=== ROBOT JOINT INFORMATION ===")
print(f"Total joints: {num_joints}\n")

joint_name_to_id = {}
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    joint_lower_limit = joint_info[8]
    joint_upper_limit = joint_info[9]
    
    joint_name_to_id[joint_name] = i
    
    # Print detailed joint info
    print(f"Joint {i}: {joint_name}")
    print(f"  Type: {joint_type}")
    print(f"  Limits: {joint_lower_limit} to {joint_upper_limit}\n")
    
    # Create slider for movable joints
    if joint_type != p.JOINT_FIXED:
        # Convert limits from radians to degrees for easier debugging
        lower_deg = math.degrees(joint_lower_limit) if joint_lower_limit != -float('inf') else -180
        upper_deg = math.degrees(joint_upper_limit) if joint_upper_limit != float('inf') else 180
        
        # Create slider
        sliders[joint_name] = p.addUserDebugParameter(
            joint_name, lower_deg, upper_deg, 0)

# Define joint mapping dictionary
# Modify this mapping based on your actual joint structure
JOINT_MAPPING = {
    "Base_Rotation": "_RotationBase",           # Joint 0: Base rotation
    "LowerHinge_Rotation": "_RotationHingeLower", # Joint 1: Lower hinge
    "UpperHinge_Rotation": "_RotationHingeUpper", # Joint 2: Upper hinge
    "EndEffector_Rotation": "_RotationUpper",    # Joint 3: End effector rotation
    
    "LowerRing_Rotation": "LongArm",            # Joint 4: Lower ring
    "LowerRing_Arm_Rotation": "LowerArmUpper",   # Joint 5: Lower ring arm
    
    "MiddleRing_Rotation": "MiddleArm",         # Joint 6: Middle ring
    "MiddleRing_Arm_Rotation": "MiddleArmUpper", # Joint 7: Middle ring arm
    
    "UpperRing_Rotation": "ShortArm",           # Joint 8: Upper ring
    "UpperRing_Arm_Rotation": "ShortArmUpper",   # Joint 9: Upper ring arm
    
    "x_rotation": "__Roll",                     # Joint 10: Platform roll
    "y_rotation": "__Pitch",                    # Joint 11: Platform pitch
    "z_rotation": "__Yaw",                      # Joint 12: Platform yaw
    
    "GripperLeft": "__GripperLeft",             # Joint 13: Left gripper
    "GripperRight": "__GripperRight"            # Joint 14: Right gripper
}

# Verify and print the mapping for debugging
print("\n=== JOINT MAPPING VERIFICATION ===")
for logical_name, urdf_name in JOINT_MAPPING.items():
    if urdf_name in joint_name_to_id:
        print(f"{logical_name} maps to URDF joint '{urdf_name}' (ID: {joint_name_to_id[urdf_name]})")
    else:
        print(f"WARNING: {logical_name} maps to '{urdf_name}' which is NOT FOUND in the URDF!")

# Debug window to display joint values
debug_text_id = p.addUserDebugText(
    text="Joint Values:\n",
    textPosition=[0, 0, 1.5],
    textColorRGB=[1, 1, 1],
    textSize=1.5
)

# Camera toggle function
current_view = "top"
camera_distance = 1.0  # Adjustable distance

def toggle_view():
    global current_view
    if current_view == "top":
        # Switch to side view
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=90, cameraPitch=0,
            cameraTargetPosition=[0, 0, 0]
        )
        current_view = "side"
    else:
        # Switch to top view
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=0, cameraPitch=-89.9,
            cameraTargetPosition=[0, 0, 0]
        )
        current_view = "top"

# Set initial camera view
p.resetDebugVisualizerCamera(
    cameraDistance=camera_distance,
    cameraYaw=45, cameraPitch=-30,  # Angled view for better debugging
    cameraTargetPosition=[0, 0, 0]
)

# Toggle between test mode and flask mode
USE_FLASK = False
test_mode_button = p.addUserDebugParameter("Toggle Test/Flask Mode", 1, 0, 0)
last_button_value = 0

# Function to fetch positions from Flask
def fetch_positions():
    while True:
        if not USE_FLASK:
            time.sleep(0.1)
            continue
            
        try:
            response = requests.get(FLASK_URL)
            if response.status_code == 200:
                raw_data = response.json()
                
                # Process data based on type
                if isinstance(raw_data, str):
                    motor_positions = raw_data.replace(",", ".").split("&")
                elif isinstance(raw_data, list):
                    motor_positions = [str(val).replace(",", ".") for val in raw_data]
                else:
                    print(f"Unexpected data format: {raw_data}")
                    continue

                if len(motor_positions) == 14:
                    # Process positions with clear labels and offsets
                    positions = {
                        # Rings
                        "UpperRing_Rotation": float(motor_positions[0]) + 120 + 30,
                        "MiddleRing_Rotation": float(motor_positions[1]) + 120 + 60,
                        "LowerRing_Rotation": float(motor_positions[2]) + 120,
                        
                        # Arm base
                        "Base_Rotation": -float(motor_positions[3]) - 180,
                        "LowerHinge_Rotation": -float(motor_positions[4]) + 30,
                        "UpperHinge_Rotation": float(motor_positions[5]) - 30,
                        "EndEffector_Rotation": -float(motor_positions[6]),
                        
                        # Gripper
                        "GripperState": float(motor_positions[7]),
                        
                        # Ring arms
                        "UpperRing_Arm_Rotation": float(motor_positions[8]),
                        "MiddleRing_Arm_Rotation": -float(motor_positions[9]),
                        "LowerRing_Arm_Rotation": -float(motor_positions[10]),
                        
                        # Platform
                        "y_rotation": -float(motor_positions[11]),
                        "z_rotation": float(motor_positions[12]),
                        "x_rotation": -float(motor_positions[13])
                    }
                    
                    # Pass to simulation
                    update_simulation_from_dict(positions)
                else:
                    print(f"Received unexpected number of values: {len(motor_positions)}")

        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")

        time.sleep(0.1)  # Poll interval

# Function to update simulation from a dictionary of values
def update_simulation_from_dict(positions_dict):
    try:
        # Debug output
        debug_text = "Joint Values:\n"
        for name, value in positions_dict.items():
            debug_text += f"{name}: {value:.2f}°\n"
        
        # Update debug text display
        p.removeUserDebugItem(debug_text_id)
        new_debug_text_id = p.addUserDebugText(
            text=debug_text,
            textPosition=[0, 0, 1.5],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )
        
        # Apply values to joints
        for logical_name, value_deg in positions_dict.items():
            # Handle special case for gripper
            if logical_name == "GripperState":
                # Apply gripper state to both gripper joints
                if "GripperLeft" in JOINT_MAPPING and JOINT_MAPPING["GripperLeft"] in joint_name_to_id:
                    left_id = joint_name_to_id[JOINT_MAPPING["GripperLeft"]]
                    p.resetJointState(robot_id, left_id, math.radians(value_deg))
                
                if "GripperRight" in JOINT_MAPPING and JOINT_MAPPING["GripperRight"] in joint_name_to_id:
                    right_id = joint_name_to_id[JOINT_MAPPING["GripperRight"]]
                    p.resetJointState(robot_id, right_id, math.radians(value_deg))
            else:
                # Handle regular joints
                if logical_name in JOINT_MAPPING:
                    urdf_name = JOINT_MAPPING[logical_name]
                    if urdf_name in joint_name_to_id:
                        joint_id = joint_name_to_id[urdf_name]
                        p.resetJointState(robot_id, joint_id, math.radians(value_deg))

    except Exception as e:
        print(f"Error updating simulation: {e}")

# Start position fetching thread
threading.Thread(target=fetch_positions, daemon=True).start()

# Main simulation loop
while True:
    # Check for button press to toggle mode
    button_val = p.readUserDebugParameter(test_mode_button)
    if button_val != last_button_value:
        USE_FLASK = not USE_FLASK
        last_button_value = button_val
        print(f"Mode switched to {'Flask' if USE_FLASK else 'Test'} mode")

    # In test mode, read values from sliders
    if not USE_FLASK:
        slider_values = {}
        for joint_name, slider_id in sliders.items():
            slider_values[joint_name] = p.readUserDebugParameter(slider_id)
        
        # Create logical mapping from URDF joint names
        logical_values = {}
        for logical_name, urdf_name in JOINT_MAPPING.items():
            if urdf_name in slider_values:
                logical_values[logical_name] = slider_values[urdf_name]
        
        # Special case for gripper - use the same value for both sides
        if "__GripperLeft" in slider_values:
            logical_values["GripperState"] = slider_values["__GripperLeft"]
        
        # Update simulation with slider values
        if logical_values:
            update_simulation_from_dict(logical_values)

    # Step simulation
    p.stepSimulation()
    
    # Check for spacebar to toggle camera view
    keys = p.getKeyboardEvents()
    if ord(' ') in keys:
        toggle_view()
    
    time.sleep(1.0 / 240)  # 240 Hz simulation