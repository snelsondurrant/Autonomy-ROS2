# Created by Nelson Durrant, Feb 2025
# 
# Simple Python script to generate straight-line intermediary waypoints
# Usage: python3 basic_path_planner.py <logging_file_path>

import sys
import yaml
import matplotlib.pyplot as plt

# Distance between intermediary waypoints (in lat/lon degrees)
# If the waypoints are too far apart, they won't be in the global costmap
# and the navigation2 stack won't be able to plan a path between them
STEP_SIZE = 0.0001

# Check the arguments for the logging file path
if len(sys.argv) < 2:
    print("Error: no logging file path provided")
    print("Usage: python3 basic_path_planner.py <logging_file_path>")
    sys.exit(1)
orig_wps_file_path = sys.argv[1]

# Read original waypoints from YAML file
try:
    with open(orig_wps_file_path, 'r') as yaml_file:
        orig_wps_data = yaml.safe_load(yaml_file)
except FileNotFoundError:
    print("Error: provided waypoints file not found")
    sys.exit(1)
except Exception as ex:
    print(f"Error: {str(ex)}")
    sys.exit(1)

# YAML object to store new waypoints
new_wps_data = {
    "waypoints": []
}

# YAML obj for temporary storage of last GPS position
last_gps_position = {
    "latitude": 0.0,
    "longitude": 0.0
}

# Generate intermediary waypoints
for leg in orig_wps_data["waypoints"]:

    if leg["leg"] == "start":
        last_gps_position["latitude"] = leg["latitude"]
        last_gps_position["longitude"] = leg["longitude"]
        new_wps_data["waypoints"].append(leg)
        continue
    else:

        # Calculate intermediary waypoints
        start_lat = last_gps_position["latitude"]
        start_lon = last_gps_position["longitude"]
        end_lat = leg["latitude"]
        end_lon = leg["longitude"]

        # Calculate the distance between the two points
        distance = ((end_lat - start_lat)**2 + (end_lon - start_lon)**2)**0.5

        # Calculate the number of intermediary waypoints
        num_waypoints = int(distance / STEP_SIZE)

        # Calculate the step size for each intermediary waypoint
        step_lat = (end_lat - start_lat) / num_waypoints
        step_lon = (end_lon - start_lon) / num_waypoints

        # Generate intermediary waypoints
        for i in range(1, num_waypoints):
            lat = start_lat + i * step_lat
            lon = start_lon + i * step_lon
            
            data = {
                "leg": leg["leg"],
                "latitude": lat,
                "longitude": lon,
                "orig_wp": False
            }
            new_wps_data["waypoints"].append(data)

        # Add the original waypoint and save as the last GPS position
        last_gps_position["latitude"] = end_lat
        last_gps_position["longitude"] = end_lon
        new_wps_data["waypoints"].append(leg)

# Plot the new waypoints in matplotlib
plt.figure()
plt.xlabel("Latitude")
plt.ylabel("Longitude")
plt.gca().invert_yaxis() # North American map
for wp in new_wps_data["waypoints"]:
    if wp["orig_wp"]:
        if wp["leg"] == "start":
            plt.plot(wp["latitude"], wp["longitude"], 'go')
        else:
            plt.plot(wp["latitude"], wp["longitude"], 'bo')
    else:
        plt.plot(wp["latitude"], wp["longitude"], 'ro')
plt.show()

# Write our new waypoints to a new YAML file
try:
    with open('../config/basic_waypoints.yaml', 'w') as yaml_file:
        yaml.dump(new_wps_data, yaml_file, default_flow_style=False)
except Exception as ex:
    print(f"Error: {str(ex)}")
    sys.exit(1)
