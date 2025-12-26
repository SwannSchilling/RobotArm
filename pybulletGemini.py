import pybullet as p
import pybullet_data
import time
import math
import requests

# === CONFIG ===
FLASK_URL = "http://127.0.0.1:5000/return_positions"
URDF_PATH = "F:\\PycharmProjects\\URDF_RobotArm\\my_robot_original\\urdf\\my_robot.urdf"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Slight gravity to keep things stable
p.setGravity(0, 0, -5) 

robot_id = p.loadURDF(URDF_PATH, useFixedBase=True)

# 1. FIND ONLY THE ARMS (Ignore the Platform/Yaw link)
link_map = {p.getJointInfo(robot_id, i)[12].decode("utf-8"): i for i in range(p.getNumJoints(robot_id))}
joint_map = {p.getJointInfo(robot_id, i)[1].decode("utf-8"): i for i in range(p.getNumJoints(robot_id))}

# The 3 Moving Arms
arm_ids = [
    link_map["upperArm_rotation"],   # Arm 0
    link_map["middleArm_rotation"],  # Arm 1
    link_map["longArm_rotation"]     # Arm 2
]

# The 3 Active Motors (Rings)
motor_ids = [
    joint_map["ShortArm"],
    joint_map["MiddleArm"],
    joint_map["LongArm"]
]

# 2. DISABLE COLLISIONS BETWEEN ARMS
# Just in case they touch, we don't want them to explode
p.setCollisionFilterPair(robot_id, robot_id, arm_ids[0], arm_ids[1], 0)
p.setCollisionFilterPair(robot_id, robot_id, arm_ids[1], arm_ids[2], 0)
p.setCollisionFilterPair(robot_id, robot_id, arm_ids[2], arm_ids[0], 0)

# 3. DEBUG SLIDERS
print("--- TRIANGLE LINK MODE ---")
print("1. Adjust 'Arm Length' to find the tips.")
print("2. Adjust 'Separation' to push the arms apart (Virtual Platform).")

sl_len = p.addUserDebugParameter("Arm Length (Z)", 0.0, 0.30, 0.132)
sl_sep = p.addUserDebugParameter("Separation (Width)", 0.0, 0.10, 0.05)
sl_erp = p.addUserDebugParameter("Stiffness (ERP)", 0.0, 1.0, 0.1)

p.resetDebugVisualizerCamera(0.4, 45, -30, [0,0,0.1])

# Storage for constraints
constraints = []

def rebuild_constraints(length, separation, stiffness):
    # Remove old
    for c in constraints:
        p.removeConstraint(c)
    constraints.clear()
    
    # We want to connect Arm 0 -> Arm 1, Arm 1 -> Arm 2, Arm 2 -> Arm 0.
    # To keep them apart (Separation), we offset the connection point.
    # We pretend the 'Socket' is slightly to the side of the arm tip.
    
    # Offset calculation (Approximate circle tangent)
    # This pushes the connection point "outward" or "sideways"
    # For a simple triangle:
    
    # CONSTRAINT 1: Arm 0 <-> Arm 1
    c1 = p.createConstraint(
        robot_id, arm_ids[0], robot_id, arm_ids[1],
        p.JOINT_POINT2POINT, [0, 0, 0],
        parentFramePosition=[0, separation, length], # Right side of Arm 0
        childFramePosition=[0, -separation, length]  # Left side of Arm 1
    )
    
    # CONSTRAINT 2: Arm 1 <-> Arm 2
    c2 = p.createConstraint(
        robot_id, arm_ids[1], robot_id, arm_ids[2],
        p.JOINT_POINT2POINT, [0, 0, 0],
        parentFramePosition=[0, separation, length],
        childFramePosition=[0, -separation, length]
    )
    
    # CONSTRAINT 3: Arm 2 <-> Arm 0
    c3 = p.createConstraint(
        robot_id, arm_ids[2], robot_id, arm_ids[0],
        p.JOINT_POINT2POINT, [0, 0, 0],
        parentFramePosition=[0, separation, length],
        childFramePosition=[0, -separation, length]
    )

    # Apply Stiffness (Softness)
    # Lower ERP = Rubber band (No explosion)
    # Higher ERP = Steel rod
    for c in [c1, c2, c3]:
        p.changeConstraint(c, maxForce=200, erp=stiffness)
        constraints.append(c)

# Initial Build
current_L = 0.132
current_S = 0.05
current_E = 0.1
rebuild_constraints(current_L, current_S, current_E)

while True:
    # 1. READ GUI
    L = p.readUserDebugParameter(sl_len)
    S = p.readUserDebugParameter(sl_sep)
    E = p.readUserDebugParameter(sl_erp)
    
    # Rebuild if changed
    if abs(L - current_L) > 0.001 or abs(S - current_S) > 0.001 or abs(E - current_E) > 0.01:
        rebuild_constraints(L, S, E)
        current_L = L
        current_S = S
        current_E = E

    # 2. DRAW DEBUG LINES (Visual Feedback)
    # Show where the tips are
    for aid in arm_ids:
        st = p.getLinkState(robot_id, aid)
        tip = p.multiplyTransforms(st[0], st[1], [0,0,L], [0,0,0,1])[0]
        p.addUserDebugLine(st[0], tip, [0,1,0], 1) # Green line = Arm

    try:
        # 3. READ FLASK DATA
        resp = requests.get(FLASK_URL, timeout=0.02)
        raw_data = resp.json()
        parts = []
        if isinstance(raw_data, str): 
            parts = [x for x in raw_data.replace(",", ".").split("&") if x.strip()]
        elif isinstance(raw_data, list): 
            parts = [str(v).replace(",", ".") for v in raw_data]

        if len(parts) >= 8:
            mp = [float(v) for v in parts]
            
            # Apply to Rings (Active Motors)
            p.resetJointState(robot_id, motor_ids[0], math.radians(-mp[0]))
            p.resetJointState(robot_id, motor_ids[1], math.radians(-mp[1]))
            p.resetJointState(robot_id, motor_ids[2], math.radians(-mp[2]))
            
            # Other joints...
            p.resetJointState(robot_id, joint_map["_RotationBase"], math.radians(-mp[3]))
            p.resetJointState(robot_id, joint_map["_RotationHingeLower"], math.radians(-mp[4]))
            p.resetJointState(robot_id, joint_map["_RotationHingeUpper"], math.radians(mp[5]))
            p.resetJointState(robot_id, joint_map["_RotationUpper"], math.radians(mp[6]))
            
            # PHYSICS STEP (This pulls the arms together)
            p.stepSimulation()

    except Exception:
        pass
    
    time.sleep(1./240.)