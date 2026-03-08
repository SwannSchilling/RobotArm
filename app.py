#!/usr/bin/python3
import time
from flask import Flask, json ,request, jsonify

import urllib.parse
import serial
import numpy as np
import math
from time import sleep
import logging
import os
import platform
import threading
app = Flask(__name__)
from flask import Flask, request
from threading import Lock

obs_lock = threading.Lock()

collect_data = True

posOffset = 0.0  # Persistent state
collect_position_data = ""
goalPositionWrist = 0
receive_position_data = ""
latest_encoder_state = [0.0] * 8  # Updated ONLY by poll_flask thread

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

# Thread-safe storage
latest_observations = {}
obs_lock = Lock()

# Define canonical order for observations
OBS_ORDER = [
    'upper_ring', 'middle_ring', 'lower_ring',
    'base', 'lower_hinge', 'upper_hinge',
    'end_effector', 'gripper'
]

@app.route("/receive_observations")
def receive_observations():
    """Robot sends observations here or queries latest observations."""
    global latest_observations

    data_string = request.args.get('data', '')

    if data_string:
        # Receiving data FROM robot
        try:
            values = [float(v.replace(',', '.')) for v in data_string.split('&')]
            if len(values) != len(OBS_ORDER):
                return f"Error: Expected {len(OBS_ORDER)} values, got {len(values)}", 400

            with obs_lock:
                latest_observations = {k: v for k, v in zip(OBS_ORDER, values)}
            return "OK", 200

        except Exception as e:
            return f"Error parsing data: {e}", 400

    else:
        # Querying stored observations
        with obs_lock:
            if not latest_observations:
                return "No observations received yet", 404
            obs_copy = latest_observations.copy()

        # Return as a predictable string
        obs_values = [obs_copy[k] for k in OBS_ORDER]
        return "&".join(f"{v:.4f}" for v in obs_values)

@app.route("/get_state", methods=["POST"])
def get_state():
    data = request.json
    act = data["act"]
    obs = data["obs"]
    return {"act": act, "obs": obs}

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
