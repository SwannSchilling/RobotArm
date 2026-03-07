#!/usr/bin/python3
import time
from flask import Flask, json ,request, jsonify
import serial
import numpy as np
import math
from time import sleep
import logging
import os
import platform
import threading
# logging.getLogger('werkzeug').setLevel(logging.ERROR)
# logging.getLogger('werkzeug').disabled = True
# Global storage for latest observations
# Storage for observations (local to THIS script)
latest_observations = {
    'upper_ring': 0.0,
    'middle_ring': 0.0,
    'lower_ring': 0.0,
    'base': 0.0,
    'lower_hinge': 0.0,
    'upper_hinge': 0.0,
    'end_effector': 0.0,
    'gripper': 0.0
}

obs_lock = threading.Lock()

collect_data = True

posOffset = 0.0  # Persistent state
collect_position_data = ""
goalPositionWrist = 0
receive_position_data = ""

def get_ip_addresses():
    system = platform.system().lower()

    if system == "linux":  # works on Raspberry Pi
        try:
            ips = os.popen("hostname -I").read().strip().split()
            if ips:
                return ips
        except Exception:
            pass

    # Fallback for Windows (and general)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return [ip]
    except Exception:
        return ["127.0.0.1"]

app = Flask(__name__)

@app.route('/poses', methods=['GET', 'POST'])
def set_pose():
    """Endpoint for web interface to set poses"""
    global pose_state
    
    if request.method == 'GET':
        # Handle GET with query parameter (your current web interface)
        msg = request.args.get('msg', '').strip()
    else:
        # Handle POST with JSON (more robust for future)
        data = request.get_json() or {}
        msg = data.get('pose', '').strip()
    
    if msg:
        pose_state["current_pose"] = msg
        pose_state["last_updated"] = time.time()
        pose_state["sequence_id"] += 1
        print(f"📝 Pose set to: {msg} (seq: {pose_state['sequence_id']})")
        return msg
    else:
        return "No pose specified", 400

@app.route('/current_pose', methods=['GET'])
def get_current_pose():
    """Endpoint for Python script to poll current pose"""
    return pose_state["current_pose"]

@app.route('/pose_status', methods=['GET'])
def get_pose_status():
    """Detailed status endpoint (useful for debugging)"""
    return jsonify({
        "current_pose": pose_state["current_pose"],
        "last_updated": pose_state["last_updated"],
        "sequence_id": pose_state["sequence_id"],
        "age_seconds": time.time() - pose_state["last_updated"] if pose_state["last_updated"] > 0 else None
    })

@app.route('/clear', methods=['GET', 'POST'])
def clear_pose():
    """Clear current pose"""
    global pose_state
    pose_state["current_pose"] = ""
    pose_state["last_updated"] = time.time()
    pose_state["sequence_id"] += 1
    print("🗑️ Pose cleared")
    return "Pose cleared"

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        "status": "healthy",
        "timestamp": time.time(),
        "has_pose": bool(pose_state["current_pose"])
    })

@app.route('/motor_command')
def motor_command():
    global goalPositionWrist
    # Return the current goal position for the wrist motor
    return str(goalPositionWrist)

@app.route('/return_positions', methods=['GET','POST'])
def return_positions():
    motorPositions = collect_position_data
    print(motorPositions)
    return json.dumps(motorPositions)

# @app.route('/receive_observations', methods=['GET','POST'])
# def receive_observations():
#     motorPositions = receive_position_data
#     print(motorPositions)
#     return json.dumps(motorPositions)

@app.route("/receive_observations")
def receive_observations():
    """Robot sends observations here"""
    global latest_observations
    
    data_string = request.args.get('data', '')
    
    if data_string:
        # Receiving data FROM robot
        try:
            values = [float(v) for v in data_string.split('&')]
            if len(values) == 8:
                with obs_lock:
                    latest_observations = {
                        'upper_ring': values[0],
                        'middle_ring': values[1],
                        'lower_ring': values[2],
                        'base': values[3],
                        'lower_hinge': values[4],
                        'upper_hinge': values[5],
                        'end_effector': values[6],
                        'gripper': values[7]
                    }
                return "OK", 200
        except Exception as e:
            return f"Error: {e}", 400
    else:
        # Querying stored observations
        with obs_lock:
            obs = latest_observations.copy()
        values = [
            obs['upper_ring'], obs['middle_ring'], obs['lower_ring'],
            obs['base'], obs['lower_hinge'], obs['upper_hinge'],
            obs['end_effector'], obs['gripper']
        ]
        return "&".join(str(round(v, 4)) for v in values)


@app.route('/set_positions/<position>', methods=['GET','POST'])
def set_positions(position):
    print(position)
    if collect_data:
        global collect_position_data
        collect_position_data = position
        return json.dumps(position)

@app.route('/debug/<debug_position>', methods=['GET','POST'])
def debug_positions(debug_position):
    print(debug_position)
    return json.dumps(debug_position)

@app.route('/')
def index():
    ip_address = request.remote_addr
    return "Requester IP: " + ip_address

@app.route('/axis_0/<ax_0>', methods=['GET','POST'])
def set_axis_0(ax_0):
    global axis_0
    axis_0 = axis_0 + int(ax_0)  
    return "axis_0 set to:"+ str(axis_0)

@app.route('/axis_1/<ax_1>', methods=['GET','POST'])
def set_axis_1(ax_1):
    global axis_1
    axis_1 = axis_1 + int(ax_1)
    return "axis_1 set to:"+ str(axis_1)

@app.route('/axis_2/<ax_2>', methods=['GET','POST'])
def set_axis_2(ax_2):
    global axis_2
    axis_2 = axis_2 + int(ax_2)
    return "axis_2 set to:"+ str(axis_2)

@app.route('/axis_3/<ax_3>', methods=['GET','POST'])
def set_axis_3(ax_3):
    global axis_3
    axis_3 = axis_3 + int(ax_3)
    return "axis_3 set to:"+ str(axis_3)

if __name__ == '__main__':
    port = 5000
    ips = get_ip_addresses()

    print("\n🌐 Flask server is running at:")
    for ip in ips:
        print(f"   → http://{ip}:{port}")

    app.run(host="0.0.0.0", port=port)
    # app.run(debug=True, port=80, host='0.0.0.0')
    # app.debug = True
    # app.run(debug=False, port=5000, host="0.0.0.0")
    # app.run(debug=True, port=8080,host="192.168.2.114")
    # app.run()

    # flask run -h 0.0.0.0 to get flask working remotely
    # cd flask_server
    # venv\Scripts\activate
    # python app.py

    # it only works in terminal and it has to be flask run --host=0.0.0.0
    # or on Linux
    # sudo flask run --host=0.0.0.0
