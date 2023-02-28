from math import pi
from tkinter.messagebox import NO
import py_trees
import numpy as np
import time
import pandas as pd
import os
import csv
from ament_index_python.packages import get_package_share_directory

# ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterType
from action_msgs.msg import GoalStatus

# messages
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from explorer_interfaces.action import Discover
from std_msgs.msg import Float32MultiArray


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        # a cartographer subscription is created to access the occupancy
        self.cartographer = CartographerSubscriber()
        rclpy.spin_once(self.cartographer)
        # grid and determine which positions to navigate to

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived at destination')
        else:
            self.get_logger().info(
                'Goal failed with status: {0}'.format(status))

        rclpy.spin_once(self.cartographer)

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # refresh the list of accessible waypoints
        rclpy.spin_once(self.cartographer)
        # grab the first waypoint
        waypoint = self.cartographer.sorted_accessible_waypoints[0]
        # pop the
        self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[
            1:]
        # first element from the list, in case the
        # accessible waypoints didn't refresh

        # write command
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = float(waypoint[0])
        goal_msg.pose.pose.position.y = float(waypoint[1])
        # goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending navigation goal request x: ' + str(round(goal_msg.pose.pose.position.x, 2)) + ' y: ' + str(
                round(goal_msg.pose.pose.position.y, 2)))

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)

        # added by i
        # _cur_pose = self._wait_for_message(
        #         self, Float32MultiArray, '/cur_robot_pose').data

        # self.get_logger().info("cur pose-goal: %.2f, %.2f" % (_cur_pose[0], _cur_pose[1]))

    def is_near(self, pose_1, pose_2):
        # '* 100' considering acc of float.
        _threshold = 0.05 * 100
        _sub = [abs(i * 100 - j * 100) for i, j in zip(pose_1, pose_2)]
        if _sub[0] <= _threshold and _sub[1] <= _threshold:
            return True
        return False

    def _distance(self, pose_1, pose_2):
        _sub = [abs(i - j) for i, j in zip(pose_1, pose_2)]
        return _sub[0] + _sub[1]

    def send_goal_pose_copy(self, target_pose: list):
        self.get_logger().info('Funt Send Goal Pose: Waiting for action server...')
        self._action_client.wait_for_server()

        # rclpy.spin_once(self.cartographer)  # refresh the list of accessible waypoints
        # self.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints  # grab the first waypoint
        # self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[1:]  # pop the
        # # first element from the list, in case the
        # # accessible waypoints didn't refresh

        _target_pose_step = [100.0, 100.0]
        while not self.is_near(pose_1=target_pose, pose_2=_target_pose_step):

            # refresh the list of accessible waypoints
            rclpy.spin_once(self.cartographer)
            # grab the first 1000 waypoint
            sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[:1000]
            # pop the
            self.cartographer.sorted_accessible_waypoints = \
                self.cartographer.sorted_accessible_waypoints[1:]
            # first element from the list, in case the
            # accessible waypoints didn't refresh

            # if target_pose in sorted_accessible_waypoints:
            #     _target_pose_step = target_pose
            # elif len(sorted_accessible_waypoints) != 1000:
            #     _target_pose_step = [.3, .3]
            # else:
            # choose a nearst pose set as 'target_pose_step'
            _distance = np.Inf
            for point in sorted_accessible_waypoints:
                if self._distance(pose_1=point, pose_2=target_pose) < _distance:
                    _distance = self._distance(
                        pose_1=point, pose_2=target_pose)
                    _target_pose_step = point
                else:
                    continue

            # write command
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = float(_target_pose_step[0])
            goal_msg.pose.pose.position.y = float(_target_pose_step[1])
            # goal_msg.pose.pose.orientation.w = 1.0

            self.get_logger().info(
                'Sending navigation goal request x: ' + str(round(goal_msg.pose.pose.position.x, 2)) + ' y: ' + str(
                    round(goal_msg.pose.pose.position.y, 2)))

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg)
            self._send_goal_future.add_done_callback(
                self.goal_response_callback)

            rclpy.spin_until_future_complete(self, self._send_goal_future)
            self.get_logger().info("spin until future")

            goal_handle = self._send_goal_future.result()
            get_result_future = goal_handle.get_result_async()

            rclpy.spin_until_future_complete(self, get_result_future)

        # _cur_pose = self._wait_for_message(
        #         self, Float32MultiArray, '/cur_robot_pose').data

        # self.get_logger().info("cur pose-goal: %.2f, %.2f" % (_cur_pose[0], _cur_pose[1]))

    def send_goal_pose(self, target_pose: list):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # refresh the list of accessible waypoints
        # rclpy.spin_once(self.cartographer)
        # grab the first waypoint
        # waypoint = self.cartographer.sorted_accessible_waypoints[0]
        waypoint = target_pose
        # pop the
        # self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[
        #     1:]
        # first element from the list, in case the
        # accessible waypoints didn't refresh

        # write command
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = float(waypoint[0])
        goal_msg.pose.pose.position.y = float(waypoint[1])
        # goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending navigation goal request x: ' + str(round(goal_msg.pose.pose.position.x, 2)) + ' y: ' + str(
                round(goal_msg.pose.pose.position.y, 2)))

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)

    def move_from_to(self, point1, point2):
        return


class CartographerSubscriber(Node):
    def __init__(self):
        super().__init__('cartographer_subscriber')
        self.occupancy_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.occupancy_callback, 10)

        self.waypoints = self.generate_list_of_waypoints(
            n_of_waypoints=100, step=0.2)
        self.accessible_waypoints = np.array([])
        self.sorted_accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible waypoints. It sorts them and
        saves them in the self.sorted_accessible_waypoints variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """

        data = np.array(msg.data)  # download the occupancy grid
        current_map_width = msg.info.width  # get the current map width
        current_map_height = msg.info.height  # get the current map height
        resolution = msg.info.resolution  # get the resolution

        # reshape the data so it resembles the map shape
        data = np.reshape(data, (current_map_height, current_map_width))

        # Here we go through every waypoint and save the ones that are accessible.
        # An accessible waypoint is one which has no obstacles, and has few or no unknown squares in the vicinity.
        self.accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])
        for waypoint in self.waypoints:
            try:
                occupancy_grid_coordinates = [int((waypoint[1] + 2.3) / resolution), int((waypoint[0] + 2.3) /
                                                                                         resolution)]
                # perform convolution
                conv = self.convolute(data, occupancy_grid_coordinates, size=9)

                # if the convolution returns True, it means the WP is accessible, so it is stored in
                # self.accessible_waypoints
                if conv[0]:
                    self.accessible_waypoints = np.append(
                        self.accessible_waypoints, waypoint)
                    self.occupancy_value = np.append(
                        self.occupancy_value, conv[1])
            # because the waypoint array is over-sized, we need to remove the values that are out of range
            except IndexError:
                pass

        # reshape the accessible waypoints array to shape (n, 2)
        self.accessible_waypoints = self.accessible_waypoints.reshape((-1, 2))

        # Sorting waypoints according to occupancy value. This allows the robot to prioritize the waypoints with
        # more uncertainty (it wont access the areas that are completely clear, thus going to the discovery frontier)
        occupancy_value_idxs = self.occupancy_value.argsort()
        self.sorted_accessible_waypoints = self.accessible_waypoints[occupancy_value_idxs[::-1]]

        # At the beginning, when all values are uncertain, we add some hardcoded waypoints so it begins to navigate
        # and has time to discover accessible areas
        if np.size(self.sorted_accessible_waypoints) == 0:
            # origion
            # self.sorted_accessible_waypoints = np.array(
            #     [[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])
            # changed by i
            self.sorted_accessible_waypoints = np.array(
                [[.3, 0.3], [0.4, .6], [-1.5, 0.0], [0.0, -1.5]])

        # Once we have the new waypoints, they are saved in self.sorted_accessible_waypoints for use by the Navigator
        # client
        self.get_logger().info('Accessible waypoints have been updated...')

    @staticmethod
    def convolute(data, coordinates, size=3, threshold=40):
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinates: the coordinates of the OccupancyGrid to convolute around
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the waypoint is accessible or not.
        :return: average: average occupancy probability of the convolution
        """
        sum = 0
        for x in range(int(coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
            for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
                # if the area is unknown, we add 100 to sum.
                if data[x, y] == -1:
                    sum += 100
                # if occupancy state is above 50 (occupied), we add 1M to the sum so that the robot DOES NOT
                # access areas near walls.
                elif data[x, y] > 50:
                    sum += 1000000
                # if the occupancy state is below 50 and known, just add the value to sum.
                else:
                    sum += data[x, y]

        # average value for the square is computed
        average = sum / (size * size)
        if average < threshold:
            # if the average of the squares is below the threshold, the waypoint is accessible
            return True, average
        else:
            # if the average is above the threshold, the waypoint has either too many unknowns, or an obstacle
            return False, average

    def generate_list_of_waypoints(self, n_of_waypoints, step):
        """

        Generates a grid of waypoints of size ('n_of_waypoints' * 'n_of_waypoints') and step size 'step'

        :param n_of_waypoints: number of total waypoints to generate per side
        :param step: float resolution of the waypoints
        :return waypoints: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of waypoints
        """

        waypoints = np.zeros((n_of_waypoints * n_of_waypoints, 2))

        i = 0
        for index_y in range(n_of_waypoints):
            for index_x in range(n_of_waypoints):
                waypoints[i] = [float(index_x) / (1/step),
                                float(index_y) / (1/step)]
                i += 1

        self.get_logger().info("Grid of waypoints has been generated.")
        return waypoints


class NavController(Node):
    """
        Added by i (28 Oct 09:51 AM)

        Control TB3 Robot by send msgs to topic '/cmd_vel'

        TFs: 
            1. Map(main, /tf_pose): 

            2. CSV: 

            # trans CSV Coord to Map: (x, y) -> (x/2 - 0/5, y/2 - 0.5) 
    """

    def __init__(self) -> None:
        super().__init__('nav_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('nav_controller initlization finished.')

    def stop(self,) -> None:
        msg_stop = Twist()
        # loop 3 times per 0.01 second, make sure to stop
        for i in range(3):
            self.cmd_vel_publisher.publish(msg_stop)
            time.sleep(0.01)

    def spin_tb3(self, spin_velocity: float, spin_time: float) -> None:
        """
            Spin Angle (rad) = spin_velocity (rad/s) * spin_time (s)
        """
        msg = Twist()
        msg.angular.z = spin_velocity
        self.cmd_vel_publisher.publish(msg)
        time.sleep(spin_time)
        self.stop()
        return

    def move_forward(self, velocity: float, move_time: float) -> None:
        """
            Move Distance (m) = velocity (m/s) * time (s)

            Note: for tb3, only speed 'x' can be set.
        """
        msg = Twist()
        msg.linear.x = velocity
        self.cmd_vel_publisher.publish(msg)
        time.sleep(move_time)
        self.stop()
        return

    def move_from_to(self, from_: list, to_: list, max_time: float) -> None:
        """
            function for outer call.
        """
        self.move_from_to_simple(from_=from_, to_=to_, max_time=max_time)
        return

    def move_from_to_simple(self, from_: list, to_: list, max_time: float) -> None:
        """
            function for inner.
            Simplest Move: from A (x1, y1) to B (x2, y2)

            Step 1: move forward (x1, y1) -> (x2, y1);
            Step 2: spin 90 degree;
            Step 3: move forward (x2, y1) -> (x2, y2);
        """

        x1, y1 = from_
        x2, y2 = to_

        # Step 1:
        distance = x2 - x1
        velocity = 0.1
        move_time = distance / velocity
        if distance > 0:
            self.move_forward(velocity=velocity, move_time=move_time)
        elif distance < 0:
            self.move_forward(velocity=-velocity, move_time=move_time)

        # Step 2:
        self.spin_tb3(spin_velocity=pi/10, spin_time=5)

        # Step 3:
        distance = y2 - y1
        velocity = 0.1
        move_time = distance / velocity
        if distance > 0:
            self.move_forward(velocity=velocity, move_time=move_time)
        elif distance < 0:
            self.move_forward(velocity=-velocity, move_time=move_time)

        return
