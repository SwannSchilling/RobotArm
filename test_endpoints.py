import requests
import time

# Configuration
OBS_URL = "http://192.168.2.122:5000/receive_observations"
POS_URL = "http://192.168.2.122:5000/return_positions"
ITERATIONS = 10
DELAY = 1  # seconds

# Canonical order (must match Flask OBS_ORDER)
OBS_ORDER = [
    'upper_ring', 'middle_ring', 'lower_ring',
    'base', 'lower_hinge', 'upper_hinge',
    'end_effector', 'gripper'
]

def fetch(url):
    """Fetch the &-separated string from Flask and return as string."""
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        return r.text.strip()
    except Exception as e:
        return f"ERROR: {e}"

def parse_obs_string(s):
    """Convert &-separated string to a dictionary with OBS_ORDER keys."""
    try:
        values = [float(v.replace(',', '.')) for v in s.split('&')]
        return {k: v for k, v in zip(OBS_ORDER, values)}
    except Exception as e:
        print(f"Error parsing string '{s}': {e}")
        return {k: None for k in OBS_ORDER}

print("\nPolling endpoints...\n")

for i in range(ITERATIONS):
    obs_str = fetch(OBS_URL)
    pos_str = fetch(POS_URL)

    obs_dict = parse_obs_string(obs_str)
    pos_dict = parse_obs_string(pos_str)

    print(f"Iteration {i+1}")
    print(f"{'Joint':>15} | {'Observation':>12} | {'Position':>12} | {'Diff':>10}")
    print("-"*60)

    for joint in OBS_ORDER:
        o = obs_dict[joint]
        p = pos_dict[joint]
        diff = o - p if o is not None and p is not None else None
        print(f"{joint:>15} | {o:12.4f} | {p:12.4f} | {diff:10.4f}")

    print("-"*60)
    time.sleep(DELAY)