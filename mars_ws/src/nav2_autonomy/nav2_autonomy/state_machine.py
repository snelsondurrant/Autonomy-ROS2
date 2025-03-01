"""
Created by Nelson Durrant, Feb 2025

State machine for the BYU Mars Rover using Nav2
Pretty cool, right?
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import SetBool
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav2_autonomy.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self, leg):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            if wp["leg"] == leg:    
                latitude, longitude = wp["latitude"], wp["longitude"] # Need to generate intermittent values somehow
                gepose_wps.append(latLonYaw2Geopose(latitude, longitude, 0.0))
        return gepose_wps


class StateMachine():
    """
    Class for running the autonomy competition task using nav2

    :author: Nelson Durrant
    :date: Feb 2025

    Subscribers:
        - *aruco poses
        - *object poses
    Services:
        - *toggle service
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator()
        self.wp_parser = YamlWaypointParser(wps_file_path)

        self.run_flag = False

        self.legs = ["start", "gps1", "gps2", "aruco1", "aruco2", "aruco3", "mallet", "bottle"]

        self.gps_legs = ["gps1", "gps2"]
        self.aruco_legs = ["aruco1", "aruco2", "aruco3"]
        self.obj_legs = ["mallet", "bottle"]

        self.hex_coord = [(2.0, 0.0), (1.0, 1.73), (-1.0, 1.73), (-2.0, 0.0), (-1.0, -1.73), (1.0, -1.73)]

        # Create the services
        self.srv = self.create_service(
            SetBool,
            "init_mission",
            self.init_callback
        )

    def listener_callback(self, request, response):
        '''
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        '''
        init_bool = request.data
        if init_bool:
            if self.started:
                response.success = False
                response.message = 'Manual Mission has already been started. Needs to be reset before initialization'
            else:
                self.get_parameters()
                self.started = request.data
                response.success = True
                response.message = 'Manual Mission Started'
                self.counter = 0
        else:
            self.started = False
            self.counter = 0
            response.success = True
            response.message = 'Manual Mission Restarted'

        return response
    
    def pose_nav(self, pose, leg):
        """
        Function to navigate to a pose
        """

        print(leg, 'Starting pose navigation')

        self.navigator.goToPose(pose)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(leg, 'Pose navigation completed')
        elif result == TaskResult.CANCELED:
            print(leg, 'Pose navigation canceled')
        elif result == TaskResult.FAILED:
            print(leg, 'Pose navigation failed')

    def gps_nav(self, leg):
        """
        Function to navigate through gps waypoints
        """

        print(leg, 'Starting gps navigation')

        wps = self.wp_parser.get_wps(leg)

        # Publish the last gps position (our goal) to mapviz
        print(wps[-1]) # TODO: publish this

        self.navigator.followGpsWaypoints(wps)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(leg, 'gps navigation completed')
        elif result == TaskResult.CANCELED:
            print(leg, 'gps navigation canceled')
        elif result == TaskResult.FAILED:
            print(leg, 'gps navigation failed')

    def spin_search(self, leg):
        """
        Function to spin in place
        """

        print(leg, 'Starting spin search')


        self.navigator.spin(spin_dist=3.14)
        while (not self.navigator.isTaskComplete()):

            if leg in self.aruco_legs:
                # Check for the aruco tag
                pose = self.aruco_check(leg)
                if pose:
                    self.navigator.cancelTask()
                    return pose

            elif leg in self.obj_legs:
                # Check for the object
                pose = self.obj_check(leg)
                if pose:
                    self.navigator.cancelTask()
                    return pose

            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(leg, 'Spin search completed')
        elif result == TaskResult.CANCELED:
            print(leg, 'Spin search canceled')
        elif result == TaskResult.FAILED:
            print(leg, 'Spin search failed')
        
        return False

    def hex_search(self, leg):
        """
        Function to search in a hex pattern
        """

        print(leg, 'Starting hex search')

        # Generate a hex pattern in the base_link frame
        for coord in self.hex_coord:
            # TODO: Convert these into gps points using the last one
            # TODO: Fix the relative frame hex error
            hex_pose = PoseStamped()
            hex_pose.header.frame_id = "base_link"
            hex_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            hex_pose.pose.position.x = coord[0]
            hex_pose.pose.position.y = coord[1]
            
            self.pose_nav(hex_pose, leg)
            found_pose = self.spin_search(leg)
            # Did the last spin search find it?
            if found_pose:
                return found_pose
        
        return False

    def aruco_check(self, leg):
        """
        Function to check for the aruco tag
        """

        return False

    def obj_check(self, leg):
        """
        Function to check for the object
        """

        # TODO: Convert these into gps points
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0

        return pose

    def exec_leg(self, leg):
        """
        Function to execute task legs
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        # Iterate through the gps legs
        if leg in self.gps_legs:

            print(leg, 'Starting gps leg')
            self.gps_nav(leg)

            # Don't wait or flash the LED for the start leg
            if leg == 'start':
                return
            
            # TODO: Wait and flash LED
            # Flashing LED can be done by publishing to a topic using the periperhals package

        # Iterate through the aruco legs
        elif leg in self.aruco_legs:

            print(leg, 'Starting aruco leg')
            self.gps_nav(leg)

            # Look for the aruco tag
            aruco_loc = self.spin_search(leg) # Do a spin search
            if not aruco_loc:
                aruco_loc = self.hex_search(leg) # Do a hex search
            if not aruco_loc:
                print(leg, 'Could not find the aruco tag')
            else:
                print(leg, 'Found the aruco tag at: ')
                print(aruco_loc)
                self.pose_nav(aruco_loc, leg)

                # TODO: Wait and flash LED

        # Iterate through the object legs
        elif leg in self.obj_legs:

            print(leg, 'Starting object leg')
            self.gps_nav(leg)

            # Look for the object
            obj_loc = self.spin_search(leg) # Do a spin search
            if not obj_loc:
                obj_loc = self.hex_search(leg) # Do a hex search
            if not obj_loc:
                print(leg, 'Could not find the object')
            else:
                print(leg, 'Found the object at: ')
                print(obj_loc)
                self.pose_nav(obj_loc, leg)

                # TODO: Wait and flash LED

    def run_state_machine(self):
        """
        Function to run the competition state machine
        """

        while True:
            while not self.run_flag:
                time.sleep(1)

            for leg in self.legs:
                if self.run_flag:
                    break
                self.exec_leg(leg)


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_autonomy"), "config", "basic_waypoints.yaml")

    nav2_sm = StateMachine(yaml_file_path)
    nav2_sm.run_state_machine()


if __name__ == "__main__":
    main()
