from turtle import shape
import rclpy
import numpy
import os
import csv
import pandas as pd
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# msgs
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


class Subscriber(Node):

    def __init__(self):
        super().__init__('watchtower')
        # Define subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'map_progress', 10)
        self.tf_pose_publisher = self.create_publisher(Point, 'tf_pose', 10)
        self.free_thresh = 0.25
        # Declare map_name parameter
        self.declare_parameter('map_name', 'map10')
        map_name_param = self.get_parameter('map_name')
        self.get_logger().info('Map selected = %s' % (str(map_name_param.value),))
        # Declare map_size parameter
        self.declare_parameter('map_size')
        map_size_param = self.get_parameter('map_size')
        # Read map file
        package_share_directory = get_package_share_directory(
            'explorer_gazebo')
        map_folder_directory = os.path.join(package_share_directory, 'maps')
        map_file = os.path.join(map_folder_directory,
                                map_name_param.value + '.csv')

        # added by i
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node=self)

        try:
            df = pd.read_csv(map_file, sep=',', header=None)
        except:
            self.get_logger().error('Could not find map file')
            raise FileNotFoundError
        sim_map_array = df.values
        sim_map_resolution = 0.5**2
        if not map_size_param.value:
            self.free_space = numpy.count_nonzero(
                sim_map_array == 0) * sim_map_resolution
        else:
            self.free_space = map_size_param.value
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # get infos from 'msg', dtype = 'OccupancyGrid'
        map_array = numpy.asarray(msg.data)
        resolution = msg.info.resolution

        try:
            self.tf_trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to base_link: {ex}')
            return
        # tf_pose = [self.tf_trans.transform.translation.x,
        #            self.tf_trans.transform.translation.y]
        self.tf_pose = Point()
        self.tf_pose.x = self.tf_trans.transform.translation.x
        self.tf_pose.y = self.tf_trans.transform.translation.y

        map_explored = numpy.count_nonzero(
            (map_array <= self.free_thresh) & (map_array > -1)) * resolution**2
        percentage_explored = map_explored/self.free_space

        map_explored_msg = Float32()

        if percentage_explored > 1.0:
            percentage_explored = 1.0
        map_explored_msg.data = percentage_explored

        self.publisher_.publish(map_explored_msg)
        self.tf_pose_publisher.publish(self.tf_pose)


def main(args=None):
    rclpy.init(args=args)

    watchtower = Subscriber()

    rclpy.spin(watchtower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchtower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
