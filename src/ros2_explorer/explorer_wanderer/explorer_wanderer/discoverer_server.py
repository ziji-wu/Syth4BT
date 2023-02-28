# written by Enrique Fernández-Laguilhoat Sánchez-Biezma and Daniel García López

#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 3rd party
import py_trees
import numpy as np
import pandas as pd
from shutil import copyfile
import os
import re
import csv
import time
from ament_index_python.packages import get_package_share_directory
from .mitl_behavior_tree import BT_Contrl, BTExecException
from .navigation_client import NavigationClient, CartographerSubscriber, NavController

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
from nav2_msgs.action import NavigateToPose
from explorer_interfaces.action import Discover


# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"


class DiscovererServer(Node):
    def __init__(self, counter_example_file: str, npz_file: str, obst_idx:int = 0):
        super().__init__('discoverer_server')
        self._action_server = ActionServer(
            self, Discover, 'discover', self.execute_callback)
        self.watchtower_subscription = self.create_subscription(
            Float32, 'map_progress', self.watchtower_callback, 10)
        self.watchtower_subscription  # prevent unused variable warning
        self.navigation_client = NavigationClient()
        self.stop_discovering = False
        self.map_completed_thres = 1.0  # Initialize threshold to max (100%)

        # do init planning and behavior tree construct.
        self.def_mitl_grammer()
        self.set_obstacles()
        self.formula = self.negop(self.final_ii(0, 120, self.phi_1))
        
        

        # before
        # self.tack_planning(formula=self.formula)
        # self.bt_ctrl = BT_Contrl(
        #     counter_example_file, npz_file, node=self, 
        #     obstacle=self.obstacles[obst_idx], 
        #     inference=self.inference[obst_idx]
        # )

        # test
        for idx in range(len(self.obstacles)):
            block = self.obstacles[idx]
            self.map_decompose()
            self.replace_parm()
            self.tack_planning(formula=self.formula)
            self.get_logger().info('reactive: blocked region (%i, %i)' % (block[0], block[1]))

            self.bt_ctrl = BT_Contrl(
                    counter_example_file, npz_file, node=self, 
                    obstacle=block, 
                    inference=self.inference[idx]
                )

            for i in range(8):
                try: 
                    self.bt_ctrl.set_bt()
                    self.get_logger().info("reactive: task success")
                    break
                except KeyboardInterrupt:
                    self.behavior_tree.interrupt()
                    break
                except BTExecException as e:
                    # self.behavior_tree.interrupt()
                    self.get_logger().info(e.error_msg)
                    succ = self.tack_replanning(e.block, cur_pose=e.cur_pose,
                                        cur_time=e.cur_time, fp_time=self.fp_time)
                    if succ == False:
                        self.get_logger().info("reactive: task failed.")
                        break
                    # self.bt_ctrl.set_bt()
                    # self.behavior_tree = self.bt_ctrl._behavior_tree
            if i == 7:
                self.get_logger().info("reactive: task failed.")

        self.behavior_tree = self.bt_ctrl._behavior_tree
        self.get_logger().info("Discoverer Server is ready")

    def watchtower_callback(self, msg):
        # If map_progress is higher than the threshold send stop wandering signal
        if msg.data > self.map_completed_thres:
            self.stop_discovering = False

    def print_tree(self, tree):
        # return
        self.get_logger().info(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    def execute_callback(self, goal_handle):
        self.get_logger().info("Discoverer Server received a goal")
        # self.map_completed_thres=goal_handle.request.map_completed_thres
        self.get_logger().info("Map completed threshold set to: %s" %
                               self.map_completed_thres)
        # Origion
        # while not self.stop_discovering:
        #     self.navigation_client.send_goal()
        # Tick BT :
        while True:
            try:
                self.behavior_tree.tick_tock(
                    period_ms=300,
                    number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
                    pre_tick_handler=None,
                    post_tick_handler=self.print_tree
                )
            except KeyboardInterrupt:
                self.behavior_tree.interrupt()
                break
            except BTExecException as e:
                self.behavior_tree.interrupt()
                self.get_logger().info(e.error_msg)
                # self.tack_replanning(block=e.block,
                #                      cur_pose=e.cur_pose,
                #                      cur_time=e.cur_time, fp_time=self.fp_time)
                # self.bt_ctrl.set_bt()
                # self.behavior_tree = self.bt_ctrl._behavior_tree

        # self.navigation_client.send_goal_pose(target_pose=[0.25, -0.25])
        # self.navigation_client.send_goal_pose(target_pose=[1.6, 2.0])

        self.get_logger().info('Job Finished')
        # goal_handle.succeed()
        return Discover.Result()

    def set_obstacles(self, ) -> None:
        self.obstacles = [
            [9, 1], [9, 4], [9, 6], [8, 1], [8, 2],
            [8, 5], [7, 1], [7, 2], [7, 3], [6, 6],
            [5, 4], [5, 6], [4, 2], [4, 3], [4, 5],
            [3, 4], [3, 6], [2, 1], [2, 3], [2, 5],
        ]
        self.inference = [
            [8], [], [4], [8], [5],
            [10], [7], [5], [5], [3],
            [7], [2], [7], [8], [],
            [], [1], [5], [9], [6],
        ]
        return

    def replace_parm(self, map_name: str = 'map1'):
        # replace parms in 'run in docker.sh'
        run_in_docker_sh_path = '/home/devel_ws/src/mitl_tack/docker/run_in_docker.sh'

        with open(run_in_docker_sh_path, 'r') as f:
            lines = f.readlines()

            for idx, line in enumerate(lines):
                if line[0] == '#':
                    continue
                if '.ta' in line:
                    lines[idx] = re.sub(r'map\d+.ta', map_name + '.ta', line)
                if '.mitli' in line:
                    lines[idx] = re.sub(
                        r'map\d+.mitli', map_name + '.mitli', line)

        with open(run_in_docker_sh_path, 'w+') as f:
            f.writelines(lines)

        return

    def def_mitl_grammer(self, ) -> None:
        """
        Useful functions for hand code mitl formula.
        """

        self.negop = lambda x: "! (%s)" % x
        self.andop = lambda x, y: '&& (%s) (%s)' % (x, y)
        self.andop3 = lambda x, y, z: '&& (%s) (&& (%s) (%s))' % (x, y, z)
        self.orop = lambda x, y: '|| (%s) (%s)' % (x, y)
        self.orop3 = lambda x, y, z: '|| (%s) (|| (%s) (%s))' % (x, y, z)
        self.imply = lambda x, y: '-> (%s) (%s)' % (x, y)

        self.final_ii = lambda x, y, z: 'F_ii %s %s (%s)' % (str(x), str(y), z)
        self.final_ee = lambda x, y, z: 'F_ee %s %s (%s)' % (str(x), str(y), z)
        self.final_iplus = lambda x, y: 'F_i+ %s (%s)' % (str(x), y)

        self.global_ii = lambda x, y, z: 'G_ii %s %s (%s)' % (
            str(x), str(y), z)
        self.global_ee = lambda x, y, z: 'G_ee %s %s (%s)' % (
            str(x), str(y), z)
        self.global_iplus = lambda x, y: 'G_i+ %s (%s)' % (str(x), y)

        self.phi_0 = 'p1_target_region_0'
        self.phi_1 = 'p1_target_region_1'
        return

    def map_decompose(self, ) -> None:
        """
            Input from specific .csv file and 
         output to specific .ta and a default .mitli file.
            1. csv file:
                /home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv
            2. ta file:
                /home/devel_ws/src/mitl_tack/map_utils/outputs/map1.ta
            3. mitli file:
                /home/devel_ws/src/mitl_tack/map_utils/outputs/map1.mitli
        """
        python = '/bin/python3'
        csv_file = '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv'
        csvMap2TA_path = '~/devel_ws/src/mitl_tack/map_utils/map_utils/csvMap2TA.py'
        map_from_csv = '/home/devel_ws/src/ros2_explorer/explorer_gazebo/gazebo-map-from-csv-copy.py'

        cmd_csvMap2TA = ' '.join([python, csvMap2TA_path, '-f', csv_file])
        cmd_genSDF = ' '.join([python, map_from_csv, '-f', 'map1.csv'])
        cmd = ' && '.join([cmd_csvMap2TA, cmd_genSDF])

        res = os.popen(cmd)
        ctx = res.read()
        res.close()

        self.get_logger().info('Execute command: %s and %s' %
                               (cmd_csvMap2TA, cmd_genSDF))
        self.get_logger().info('Execute Output: %s' % ctx)
        return

    def tack_planning(self, formula: str) -> bool:
        """
            Input from specific files. Directly change those files 
        if want to change function input.
            1. ta file: 
                /home/devel_ws/src/mitl_tack/map_utils/outputs/map1.ta
            2. mitli file:
                /home/devel_ws/src/mitl_tack/map_utils/outputs/map1.mitli

            !note: need to deal with exception 

            Return:
                A bool value, planning success or fail.
        """

        def replace_formula(formula: str) -> None:
            """
                Replace formula in /home/devel_ws/src/mitl_tack/map_utils/outputs/map1.mitli
            with 'formula'
            """
            mitli_file = '/home/devel_ws/src/mitl_tack/map_utils/outputs/map1.mitli'
            with open(mitli_file, 'w+') as f:
                f.write(formula)
            return

        def stop_planning():
            """
                Stop Docker
            """
            res = os.popen('sudo docker ps')
            ctx = res.read().split()
            res.close()
            self.get_logger().info(' '.join(ctx))

            if len(ctx) > 8:
                os.system('sudo docker stop ' + ctx[8])
                self.get_logger().info(
                    'Stopped Docker Planner with ID: %s' % ctx[8])
            else:
                self.get_logger().info('All Docker Planner Stopped')
            return

        cmd_docker = ''.join(['echo %s | ' % '0', 'sudo docker run ', '--rm --privileged ',
                              '-v ~/devel_ws/src/mitl_tack/:/workspace/mitl_tack ',
                              '--name tack ', 'i/tack ', 'bash /workspace/onStart.sh ',
                              '-s -- -p x86_64'])

        replace_formula(formula=formula)
        self.get_logger().info('Execute command: %s ' % cmd_docker)
        stop_planning()
        try:
            start = time.perf_counter()
            res = os.popen(cmd_docker)
            ctx = res.read()
            res.close()
            self.get_logger().info('Execute Output: %s' % ctx)
        except Exception:
            pass
        # os.system(cmd_docker)

        # find and set fp_time
        # fp_time_pattern = re.compile(r'\d+ ms')
        # self.fp_time = float(fp_time_pattern.findall(ctx)[0][:-2]) / 1000
        self.fp_time = time.perf_counter() - start

        self.get_logger().info('reactive: Set first planning time: %.3f' % self.fp_time)
        stop_planning()

        if 'not satisfied' in ctx:
            return True
        elif 'satisfied' in ctx:
            self.get_logger().warning("TACK Planning Failed.")
            return False

        return

    def tack_replanning(self, block: str, cur_pose: str, cur_time: int, fp_time: int):
        """
            Parm:
                cur_pose:
                cur_time:
                first_planning_time:

            Do:
                1. replace 'init state' in ta file with cur_pose
                2. update mitl formula using cur_time & fp-time
                3. call 'self.tack_planning()'
        """
        def replace_initPose(cur_pose: str, block: str):
            ta_path = '/home/devel_ws/src/mitl_tack/map_utils/outputs/map1.ta'
            with open(ta_path, 'r') as ta_file:
                ta = ta_file.read()
                ta = re.sub(r'init .+;', 'init ' + cur_pose + ';', ta)
            with open(ta_path, 'w+') as ta_file:
                ta_file.write(ta)
            self.get_logger().info('Replace Init Pose in ta with pose: %s' % cur_pose)
            return

        def remove_blocked_inTA(block: str) -> None:
            ta_path = '/home/devel_ws/src/mitl_tack/map_utils/outputs/map1.ta'
            with open(ta_path, 'r') as ta_file:
                ta_lines = ta_file.readlines()
                new_lines = []
                for line in ta_lines:
                    if block in line:
                        if 'state' in line:
                            new_lines.append(line[:9])
                        elif 'trans' in line:
                            new_lines.append(line[:9])
                        else:
                            pass
                    else:
                        new_lines.append(line)

            with open(ta_path, 'w+') as ta_file:
                ta_file.writelines(new_lines)

            return

        def new_formula(cur_time: int, fp_time: int) -> None:
            # case1: F_ii lt ut or F_ee lt ut
            #       or G_ii lt ut or G_ee lt ut
            # case2: F_i+ lt or F_e+ lt or G_i+ lt or G_e+ lt

            # formula = '! (&& (F_ii 0 120 (p1_target_region_1)) (&& (F_i+ 120 (p1_target_region_0)) (F_i+ 0 (p1_target_region_2))))'

            cost = int(cur_time + fp_time)
            # if cost % 10 != 0:
            #     cost += (10 - cost % 10)

            num_pattern = re.compile(r'\d+')
            case1_pattern = re.compile(r'(F|G)\_(ii|ee) \d+ \d+')
            case2_pattern = re.compile(r'(F|G)\_(i\+|e\+) \d+')

            for match in case1_pattern.finditer(self.formula):
                match = match.group()
                times = [int(i) for i in num_pattern.findall(match)]
                times = [max(0, i - cost) for i in times]

                if times[1] == 0:
                    # todo: task failed, raise error
                    pass
                else:
                    self.formula = re.sub(match,
                                          match[:5] + str(times[0]) + ' ' + str(times[1]), self.formula)

            for match in case2_pattern.finditer(self.formula):
                match = match.group()
                times = [int(i) for i in num_pattern.findall(match)]
                times = [max(0, i - cost) for i in times]

                match_re = re.sub(r'\+', '\+', match)
                self.formula = re.sub(match_re,
                                      match[:5] + str(times[0]), self.formula)

            self.get_logger().info('Set new formula: %s' % self.formula)

            return

        replace_initPose(cur_pose=cur_pose, block=block)
        remove_blocked_inTA(block)
        new_formula(cur_time=cur_time, fp_time=fp_time)
        self.get_logger().info("reactive: start do replanning")
        
        return self.tack_planning(formula=self.formula)


def main(args=None):
    rclpy.init(args=args)

    counter_example_file = '/home/devel_ws/src/mitl_tack/docker/result/counterexample.txt'
    npz_file = '/home/devel_ws/src/mitl_tack/map_utils/outputs/map1.npz'
    discoverer_server = DiscovererServer(counter_example_file, npz_file, obst_idx=0)

    rclpy.spin(discoverer_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
