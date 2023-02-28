"""
    This file Created by i (i@)

    @ Describe: A behavior tree Control Class. Using Python Lib: 'py_trees'

    @ Main Class: 'BT_Contrl'

        1. Partical Attributes (Important):
            'self._counter_example_file':
            'self._behavior_tree':

            Gen in '_parseTxt2HoareTriple(self)':
                'self._counter_example' (str): Raw str from txt file.
                'self._hoare_triple' (list[list[str,str], ...]):
                        Example:
                            [[decomp_region_0, decomp_region_1], [
                                decomp_region_1, target_region_0], ...]
                'self._time_during' (list[float]):

            Gen in '_parseNpzToAttachInfo(self)':
                1. 'self._regions':
                2. 'self._target_regions':
                3. 'self._region_decomp':
                4. 'self._conn_decomp_decomp':
                5. 'self._conn_decomp_target':
                6. 'self._guard_decomp':
                7. 'self._guard_target':

        2. Inner Class (subclass of 'py_trees.behaviour.Behaviour'):
            'Behavior_Location_Check':
            'Behavior_Move_Robot':

    Class will be Instantiated at file:
        '/home/devel_ws/src/ros2_explorer/explorer_wanderer/explorer_wanderer/discoverer_server.py'

"""


import sys
sys.path.append(
    '/home/devel_ws/src/ros2_explorer/explorer_wanderer/explorer_wanderer/')
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import numpy as np
import py_trees
import logging
import rclpy
import math
import time
from asyncio.log import logger
import re
import os
import networkx as nx
from navigation_client import NavigationClient, NavController


# msgs
# self code


class BT_Contrl(object):

    def __init__(self, counter_example_file: str, npz_file: str, node: Node,
                 obstacle: list, inference: list) -> None:
        """
            Method '__init__':
                    @ Input: Txt of Counter Example. (Output of TACK if '!Prop' NOT Satisfied.)
                        Path: '/home/devel_ws/src/mitl_tack/docker/result/counterexample.txt'
                        NOTE: Path may not exist.
                    @ Process:
                        Read counter example txt -> _parseTxt2HoareTriple (self._counter_example/_hoare_triple/_time_during)
                            -> construct Action -> construct BT.
        """

        # Set Logger
        logging.basicConfig(
            level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        logger = logging.getLogger("behavior_tree.py")

        self._counter_example_file = counter_example_file
        self._attach_info_file = npz_file
        self.controller = NavigationClient()
        self.node = node
        self.global_start_time = self.node.get_clock().now().nanoseconds / 10 ** 9
        self.obstacle = obstacle
        self.inference = inference

        # call sequence can not be change
        self.parseNpzToAttachInfo()
        # self.set_bt()
        
        # self.new_behavior_tree()
        # self.new_nehavior_tree_nav()
        logger.info("Behavior Tree Control Init Finished.")

    def set_bt(self, ) -> None:
        self.parseCE2Hoare_verify()
        success = self.replanning_with_huristic()
        
        self.new_behavior_tree()
        logger.info('New BT setted.')
        return

    def replanning_with_huristic(self) -> bool:
        # infos
        if len(self.inference) == 0:
            logger.info('reactive: task success.')
            return True
        else:
            blocked = 'decomp_region_' + str(self.inference[0])
            self.graph.remove_node(blocked)

            for idx, tmp in enumerate(self._hoare_triple[:-1]):
                if tmp[1] == blocked:
                    path, weight = self.mini_search(tmp[0], self._hoare_triple[idx+1][1])
                    except_weight = self._time_during[idx] + self._time_during[idx + 1]
                    
                    if len(path) == 0 or not (weight/except_weight < 1.2):
                        # do replanning
                        raise BTExecException(1, blocked, tmp[0], self.node.get_clock().now().nanoseconds / 10 ** 9)
                    else:
                        return True
                        # todo: substitude subtree with new path
                else:
                    continue

        return

    def replanning_without_huristic(self) -> bool:
        # infos
        if len(self.inference) == 0:
            logger.info('reactive: task success.')
            return True
        else:
            blocked = 'decomp_region_' + str(self.inference[0])
            for idx, tmp in enumerate(self._hoare_triple):
                if tmp[1] == blocked:
                    # do replanning
                    # logger.info('reactive: do replanning ')
                    raise BTExecException(1, blocked, tmp[0], self.node.get_clock().now().nanoseconds / 10 ** 9)
    
    def no_replanning_without_huristic(self, ) -> bool:
        # infos
        if len(self.inference) == 0:
            logger.info('reactive: task success.')
            return True
        else:
            blocked = 'decomp_region_' + str(self.inference[0])
            for idx, tmp in enumerate(self._hoare_triple):
                if tmp[1] == blocked:
                    return False                    

    def mini_search(self, init_pose: str, end_pose: str) -> list:
        # test
        # logger.info(' '.join(nx.dijkstra_path(self.graph, 'target_region_0', 'target_region_1')))
        start = time.perf_counter()
        path = nx.dijkstra_path(self.graph, init_pose, end_pose)
        logger.info("reactive: Mini search used %.8f s" % (time.perf_counter() - start))
        weight = nx.dijkstra_path_length(self.graph, init_pose, end_pose)

        return [[i, j] for i, j in zip(path[:-1], path[1:])], weight

    def load_timeRegions_fromTxt(self, from_loop=True) -> list:
        with open(self._counter_example_file, 'r') as txt_file:
            self._counter_example = txt_file.readlines()

        loop_idx = self._counter_example.index('**LOOP**\n')

        now_pattern = re.compile(r'now=\d+\.\d+')
        region_pattern = re.compile(r'p1\..+')

        if from_loop:
            ce_string = ''.join(self._counter_example[loop_idx - 5:])
        else:
            ce_string = ''.join(self._counter_example[:])

        times = [float(i[4:]) for i in re.findall(now_pattern, ce_string)]
        regions = [i[3:] for i in re.findall(region_pattern, ce_string)]

        return [times, regions]

    def parseCE2Hoare_v1(self) -> None:
        """
            Use Parm: 'self._counter_example_file'
            Generate Parm:
                    'self._counter_example' (str):
                    'self._hoare_triple' (list[list[str,str], ...]):
                        Example:
                            [[decomp_region_0, decomp_region_1], [
                                decomp_region_1, target_region_0], ...]
                    'self._time_during' (list[float]):

        """

        times, regions = self.load_timeRegions_fromTxt()

        self._time_during = []
        [self._time_during.append(j - i) for i, j in zip(times, times[1:])]
        self._hoare_triple = []

        for i in range(len(regions) - 1):
            self._hoare_triple.append([regions[i], regions[i+1]])
        self.change_hoare_sequence()

    def parseCE2Hoare_v2(self) -> None:
        """
            Use Parm: 'self._counter_example_file'
            Generate Parm:
                    'self._counter_example' (str):
                    'self._hoare_triple' (list[list[str,str], ...]):
                        Example:
                            [[decomp_region_0, decomp_region_1], [
                                decomp_region_1, target_region_0], ...]
                    'self._time_during' (list[float]):

        """

        times, regions = self.load_timeRegions_fromTxt()

        self._time_during = []
        [self._time_during.append(j - i) for i, j in zip(times, times[1:])]
        self._hoare_triple = []

        for i in range(len(regions) - 1):
            self._hoare_triple.append([regions[i], regions[i+1]])

        # self._hoare_triple.append([regions[-1], regions[0]])

        # print(times)
        # print(regions)
        # print(self._time_during)
        # print(self._hoare_triple)

        self.optimize_hoare_triple()
        self.change_hoare_sequence()

    def parseCE2Hoare_verify(self, ltl=False) -> None:
        times, regions = self.load_timeRegions_fromTxt(from_loop=False)

        # do clean
        if not ltl:
            tmp_time, tmp_reg = [], []
            for idx, region in enumerate(regions):
                if 'target_region_0' in region and not (40 <= times[idx] <= 80) and not ltl:
                    tmp_time[-1] = times[idx]
                elif 'target_region_1' in region and not (80 <= times[idx] <= 120) and not ltl:
                    tmp_time[-1] = times[idx]
                else:
                    tmp_time.append(times[idx])
                    tmp_reg.append(region)
            times, regions = tmp_time, tmp_reg

        self._time_during = []
        self._hoare_triple = []
        [self._time_during.append(j - i) for i, j in zip(times, times[1:])]
        [self._hoare_triple.append([regions[i], regions[i+1]]) for i in range(len(regions) - 1)]

        if not ltl:
            self.optimize_hoare_triple()

        return

    def optimize_hoare_triple(self):

        # init op != 0
        operate = 1
        while operate != 0:
            tmp_hoare = []
            tmp_time = []
            operate = 0
            last_operated = False
            idx = 0

            while idx < len(self._hoare_triple) - 1:
                before = self._hoare_triple[idx]
                after = self._hoare_triple[idx+1]
                if (before[1] == after[0] and after[0] == after[1])\
                    or (before[0] == before[1] and before[1] == after[0]):
                    tmp_hoare.append([before[0], after[1]])
                    tmp_time.append(
                        self._time_during[idx] + self._time_during[idx+1])

                    if idx + 1 == len(self._hoare_triple) - 1:
                        last_operated = True
                    operate += 1
                    idx += 2
                elif before[1] == after[0] and before[0] == after[1] and 'target_region' not in before[1]:
                    tmp_hoare.append([before[0], after[1]])
                    tmp_time.append(
                        self._time_during[idx] + self._time_during[idx+1])
                    if idx + 1 == len(self._hoare_triple) - 1:
                        last_operated = True
                    operate += 1
                    idx += 2
                elif before[0] == after[0] and before[1] == after[1]:
                    idx += 1
                    continue
                else:
                    tmp_hoare.append(before)
                    tmp_time.append(self._time_during[idx])
                    idx += 1
            if last_operated == True:
                self._hoare_triple = tmp_hoare
                self._time_during = tmp_time
            else:
                self._hoare_triple = tmp_hoare + self._hoare_triple[-1:]
                self._time_during = tmp_time + self._time_during[-1:]
        return

    def change_hoare_sequence(self,) -> None:
        min_node_idx = 0
        min_distance = 1E20

        def _distance(list1, list2):
            diff = [(i - j) ** 2 for i, j in zip(list1, list2)]
            return math.sqrt(sum(diff))

        for idx, triple in enumerate(self._hoare_triple):
            dist = _distance(self.strLoc2_numLoc(triple[0]), [0.0, 0.0])
            if dist < min_distance:
                min_distance = dist
                min_node_idx = idx

        self._hoare_triple = self._hoare_triple[min_node_idx:] + \
            self._hoare_triple[:min_node_idx]

        return

    def parseNpzToAttachInfo(self) -> None:
        """
            Use Parm:
                'self._attach_info_file'

            Infos in npz Attachment:
                1. regions:
                2. target_regions:
                3. region_decomp:
                4. conn_decomp_decomp:
                5. conn_decomp_target:
                6. guard_decomp:
                7. guard_target:

            To -> :
                1. self._regions:
                2. self._target_regions:
                3. self._region_decomp:
                4. self._conn_decomp_decomp:
                5. self._conn_decomp_target:
                6. self._guard_decomp:
                7. self._guard_target:
        """

        _npz_info = np.load(self._attach_info_file, allow_pickle=True)

        self._regions = _npz_info['regions']
        self._target_regions = _npz_info['target_regions']
        self._region_decomp = _npz_info['region_decomp']
        self._conn_decomp_decomp = _npz_info['conn_decomp_decomp']
        self._conn_decomp_target = _npz_info['conn_decomp_target']
        self._guard_decomp = _npz_info['guard_decomp']
        self._guard_target = _npz_info['guard_target']

        self.graph = nx.DiGraph()
        self.graph.add_nodes_from(
            ['target_region_' + str(i) for i in range(len(self._target_regions))] + ['decomp_region_' + str(i) for i in range(len(self._region_decomp))]
        )

        for i in self._conn_decomp_decomp:
            self.graph.add_edge('decomp_region_' + str(i[0]), 'decomp_region_' + str(i[1]), weight=self._guard_decomp[i[0]])
            self.graph.add_edge('decomp_region_' + str(i[1]), 'decomp_region_' + str(i[0]), weight=self._guard_decomp[i[1]])
        for i in self._conn_decomp_target:
            self.graph.add_edge('decomp_region_' + str(i[0]), 'target_region_' + str(i[1]), weight=self._guard_decomp[i[0]])
            self.graph.add_edge('target_region_' + str(i[1]), 'decomp_region_' + str(i[0]), weight=self._guard_target[i[1]])
        
        return

    def strLoc2_numLoc(self, strLoc: str) -> np.ndarray:
        """
            Describe: 1. str location(e.g. "")
                            to number type location (e.g. "", dtype = np.ndarray[float])
                      2. tf translate.
        """

        if 'decomp_region_' in strLoc:
            decomp_idx = int(strLoc.split('_')[-1])
            coord_init = self._region_decomp[decomp_idx]
        elif 'target_region_' in strLoc:
            target_idx = int(strLoc.split('_')[-1])
            coord_init = self._target_regions[target_idx][::2]
        else:
            logger.warn("Error strLoc Format!")
            return []

        # tf translate
        # swap x and y
        # logger.info('raw data (%.2f, %.2f), (%.2f, %.2f)' % (
        #     coord_init[0][0], coord_init[0][1], coord_init[1][0], coord_init[1][1]))
        # no question here, tf is correct
        coord_init_center = [(coord_init[0][1] + coord_init[1][1]) / 2,
                             (coord_init[0][0] + coord_init[1][0]) / 2]

        meterPerPixel = 1 / 2  # [meter/pixel]

        return [i * meterPerPixel - 0.5 for i in coord_init_center]

    def numLoc2_strLoc(self, numLoc: list) -> str:
        numLoc = [(i+0.5) * 2 for i in numLoc]
        for idx, tmp in enumerate(self._target_regions):
            tmp = tmp[::2]
            xmax, xmin = max(tmp[0][0], tmp[1][0]), min(tmp[0][0], tmp[1][0])
            ymax, ymin = max(tmp[0][1], tmp[1][1]), min(tmp[0][1], tmp[1][1])
            if xmin <= numLoc[0] <= xmax and ymin <= numLoc[1] <= ymax:
                return 'target_region_' + str(idx)

        for idx, tmp in enumerate(self._region_decomp):
            xmax, xmin = max(tmp[0][0], tmp[1][0]), min(tmp[0][0], tmp[1][0])
            ymax, ymin = max(tmp[0][1], tmp[1][1]), min(tmp[0][1], tmp[1][1])
            if xmin <= numLoc[0] <= xmax and ymin <= numLoc[1] <= ymax:
                return 'decomp_region_' + str(idx)
        return

    def new_cond_node(self, name: str, target_loc: str, node: Node):
        _target_pose = self.strLoc2_numLoc(strLoc=target_loc)
        tmp_node = self.Behavior_Check_Pose(
            name=name,
            check_pose=_target_pose,
            node=node,
            global_start_time=self.global_start_time,
            funt_wait_for_msg=self.wait4message,
            funt_is_in_location=self.is_near,
            funt_get_pose=self.get_tf2_pose
        )
        tmp_node.set_exceptionHandle(self.raise_exception)
        return tmp_node

    def new_act_node(self, name: str, target_loc: str, max_time:float, node: Node):
        _target_pose = self.strLoc2_numLoc(strLoc=target_loc)
        tmp_node = self.Behavior_TB3_Move(
            name=name,
            pose_move_to=_target_pose,
            node=node,
            global_start_time=self.global_start_time,
            max_time=max_time,
            funt_wait_for_msg=self.wait4message,
            funt_is_in_location=self.is_near,
            funt_get_pose=self.get_tf2_pose,
            controller=self.controller
        )
        tmp_node.set_exceptionHandle(self.raise_exception)
        return tmp_node

    def new_behavior_tree(self) -> None:
        """
            Use Parm:
                'self._hoare_triple'

            Gen Parm:
                'self._behavior_tree'
        """

        root = py_trees.composites.Sequence('Sequence(root)', memory=True)
        root.add_child(self.new_act_node(name='ActNode(move to start)',
                       target_loc=self._hoare_triple[0][0], max_time=5.0, node=self.node))

        for idx, triple in enumerate(self._hoare_triple):
            # todo: may need to force those excuted condition node to be True, otherwise, next Tick error.
            cond_node = self.new_cond_node(
                name='CondNode(%s)' % triple[0], target_loc=triple[0], node=self.node)
            action_node = self.new_act_node(
                name='ActNode(%s)' % triple[1], 
                target_loc=triple[1], 
                max_time=self._time_during[idx], 
                node=self.node
            )

            sub_root = py_trees.composites.Sequence('Sequence', memory=False)
            sub_root.add_children([cond_node, action_node])

            root.add_child(sub_root)

        self._behavior_tree = py_trees.trees.BehaviourTree(root=root)

        # print behavior tree
        logger.info(py_trees.display.unicode_tree(root=root))

        logger.info("Behavior Tree Successfully Generated.")

        return

    def new_nehavior_tree_nav(self) -> None:
        root = py_trees.composites.Sequence('Sequence(root)', memory=True)
        root.add_child(self.new_act_node(name='ActNode(move to start)',
                       target_loc='target_region_0', max_time=5.0, node=self.node))
        
        cond_node = self.new_cond_node(
                name='CondNode(target_region_0)', target_loc='target_region_0', node=self.node)
        action_node = self.new_act_node(
            name='ActNode(target_region_1)', 
            target_loc='target_region_1', 
            max_time=5, 
            node=self.node
        )
        sub_root = py_trees.composites.Sequence('Sequence', memory=False)
        sub_root.add_children([cond_node, action_node])
        root.add_child(sub_root)

        cond_node = self.new_cond_node(
                name='CondNode(target_region_1)', target_loc='target_region_1', node=self.node)
        action_node = self.new_act_node(
            name='ActNode(target_region_0)', 
            target_loc='target_region_0', 
            max_time=5, 
            node=self.node
        )

        sub_root = py_trees.composites.Sequence('Sequence', memory=False)
        sub_root.add_children([cond_node, action_node])
        root.add_child(sub_root)

        self._behavior_tree = py_trees.trees.BehaviourTree(root=root)

        return

    """
        # Common Functions as Parms for those two Inner Classes.
    """

    def is_near(self, pose_1, pose_2, threshold=0.3) -> bool:

        # '* 100' considering acc of float.
        threshold = threshold * 100
        sub = [abs(i * 100 - j * 100) for i, j in zip(pose_1, pose_2)]
        if sub[0] <= threshold and sub[1] <= threshold:
            return True
        return False

    def wait4message(self, node, topic_type, topic):
        """
            Function from: 
                https://fishros.org.cn/forum/topic/16/ros2%E4%B8%ADwait_for_message-%E5%87%BD%E6%95%B0%E5%AE%9E%E7%8E%B0%E6%96%B9%E6%B3%95

            Get latest msg from topic.
        """
        class _vfm(object):
            def __init__(self) -> None:
                self.msg = None

            def cb(self, msg):
                self.msg = msg

        vfm = _vfm()
        subscription = node.create_subscription(
            topic_type, topic, vfm.cb, 1)
        repeat_times = 150
        while rclpy.ok() and repeat_times > 0:
            if vfm.msg != None:
                return vfm.msg
            rclpy.spin_once(node)
            time.sleep(0.001)
            repeat_times -= 1
        # unsubcription
        subscription.destroy()

    # ros2 topic pub --rate 1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

    def get_tf2_pose(self, node: Node, ) -> list:
        pose = self.wait4message(node, Point, '/tf_pose')
        return [pose.x, pose.y]

    def raise_exception(self, global_start_time: float, node: Node):
        error_code = 1
        cur_time = node.get_clock().now().nanoseconds / (10 ** 9)
        cur_time = int(cur_time - global_start_time)

        list_pose = self.get_tf2_pose(node=node)
        cur_pose = self.numLoc2_strLoc(numLoc=list_pose)

        raise BTExecException(error_code, block='decomp_region_' + str(self.inference[0]), cur_pose=cur_pose, cur_time=cur_time)

    """
        # Same as Behavior Libs
        # Inner Class (subclass of 'py_trees.behaviour.Behaviour')
    """

    class Behavior_Check_Pose(py_trees.behaviour.Behaviour):
        """
            Condition Node
        """

        def __init__(self, name: str, check_pose: list, node: Node,
                    global_start_time: float, 
                     funt_is_in_location, funt_wait_for_msg, funt_get_pose):
            super().__init__(name)
            self.check_pose = check_pose
            self.node = node
            self.global_start_time = global_start_time
            self.is_near = funt_is_in_location
            self.wait_for_msg = funt_wait_for_msg
            self.get_pose = funt_get_pose

        def set_exceptionHandle(self, exceptionHandle):
            self.raise_exception = exceptionHandle

        # def initialise(self):
        #     # logger.info("  %s [CondNode::initialise()]" % self.name)
        #     # _cur_pose = self.wait_for_msg(
        #     #     self.node, Float32MultiArray, '/cur_robot_pose').data
        #     # self.cur_pose = np.asarray(_cur_pose)

        #     _cur_pose = self.wait_for_msg(
        #         self.node, PoseWithCovarianceStamped, '/amcl_pose').pose.pose.position
        #     self.cur_pose = np.asarray([_cur_pose.x, _cur_pose.y])

        #     logger.info("CondNode: Cur pose: x - %.2f, y - %.2f" %
        #                 (self.cur_pose[0], self.cur_pose[1]))
        #     logger.info("CondNode: Target pose: x - %.2f, y - %.2f" %
        #                 (self.check_pose[0], self.check_pose[1]))

        def update(self):
            """
                Here check whether Robot at 'self.location'.
                And return 'py_trees.common.Status.RUNNING'
                        or 'py_trees.common.Status.SUCCESS'
                        or 'py_trees.common.Status.FAILURE'

                Steps:
                    Recv '/tf_pose'
                        -> (success) -> check is near -> return success/failure;
                        -> (failure) -> spin robot -> return running;
            """
            super().update()
            self.tb3_pose = self.get_pose(self.node)
            logger.info("CondNode: tb3 current pose (%.2f, %.2f) and check pose (%.2f, %.2f)" %
                        (self.tb3_pose[0], self.tb3_pose[1], self.check_pose[0], self.check_pose[1]))

            if self.is_near(self.tb3_pose, self.check_pose, threshold=0.3):
                cur_time = self.node.get_clock().now().nanoseconds / 10 ** 9
                logger.info("%f, TB3 Success in pose %s" %
                            (cur_time, self.name))
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

    class Behavior_TB3_Move(py_trees.behaviour.Behaviour):
        """
            Act Node
        """

        def __init__(self, name: str, pose_move_to: list, node: Node,
                     global_start_time: float, max_time: float,
                     funt_is_in_location, funt_wait_for_msg, funt_get_pose, controller):
            super().__init__(name)
            self.target_pose = pose_move_to
            self.node = node
            self.global_start_time = global_start_time
            self.max_time = max_time
            self.is_near = funt_is_in_location
            self.wait_for_msg = funt_wait_for_msg
            self.get_pose = funt_get_pose
            self.controller = controller
        
        def set_exceptionHandle(self, exceptionHandle):
            self.raise_exception = exceptionHandle

        def initialise(self):
            self.raise_exception(self.global_start_time, self.node)
            self.local_start_time = self.node.get_clock().now().nanoseconds / 10 ** 9
            self.pose = self.get_pose(self.node)
            logger.info('%s: move tb3 from (%.2f, %.2f) to (%.2f, %.2f).' % (self.name.replace('\n', ''),
                                                                             self.pose[0], self.pose[1],
                                                                             self.target_pose[0], self.target_pose[1]))

            # self.controller.send_goal_pose(target_pose=self.target_pose)

            self.controller.send_goal_pose(
                target_pose=self.set_goal_bias(bias=0.08))

            self.finish_time = self.node.get_clock().now().nanoseconds / 10 ** 9

            if (self.finish_time - self.local_start_time) < self.max_time:
                time.sleep(self.max_time -
                           (self.finish_time - self.local_start_time))
            # self.pose = self.get_pose(self.node)
            # logger.info("ActNode(%s): pose x - %.2f, y - %.2f" %
            #             (self.name, self.pose[0], self.pose[1]))

            # _cur_pose = self.wait_for_msg(
            #     self._node, Float32MultiArray, '/cur_robot_pose').data
            # self.cur_pose = np.asarray(_cur_pose)

            # logger.info("ActNode: Cur Pose - (%.2f, %.2f); Target Pose - (%.2f, %.2f)" %
            #             (self.cur_pose[0], self.cur_pose[1], self._pose_move_to[0], self._pose_move_to[1]))

        def set_goal_bias(self, bias=0.12) -> list:
            goal_biased = self.target_pose
            diff = [self.pose[i] - self.target_pose[i] for i in range(2)]

            if diff[0] >= 0:
                goal_biased[0] -= bias
            else:
                goal_biased[0] += bias

            if diff[1] >= 0:
                goal_biased[1] -= bias
            else:
                goal_biased[1] += bias

            return goal_biased

        def update(self):
            """
                Here Control Robot Move to 'self.point_move_to'.
                And return 'py_trees.common.Status.RUNNING'
                        or 'py_trees.common.Status.SUCCESS'
                        or 'py_trees.common.Status.FAILURE'
            """
            super().update()
            # self.pose = self.get_pose(self.node)

            # self.controller.move_from_to(
            #     self.pose, self.target_pose, max_time=10)

            # _cur_pose = self.wait_for_msg(
            #     self._node, Float32MultiArray, '/cur_robot_pose').data
            # _cur_pose = np.asarray(_cur_pose)

            # logger.info("ActNode: Cur Pose - (%.2f, %.2f); Target Pose - (%.2f, %.2f)" %
            #             (_cur_pose[0], _cur_pose[1], self._pose_move_to[0], self._pose_move_to[1]))
            # if self.is_in_location(self.cur_pose, self._pose_move_to):
            #     logger.info("ActNode: success in pose (%.2f, %.2f)" %
            #                 (self.cur_pose.data[0], self.cur_pose.data[1]))
            #     return py_trees.common.Status.SUCCESS
            # else:
            #     # logger.info("send goal pose (%.2f, %.2f) to navigation client" % (
            #     #     self._pose_move_to[0], self._pose_move_to[1]))
            #     # self.navigation_client.send_goal_pose(
            #     #     target_pose=self._pose_move_to)
            #     return py_trees.common.Status.FAILURE
            # return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS


class BTExecException(Exception):
    def __init__(self, error_code: int, block: str, cur_pose: str, cur_time: int):
        self.error_code = error_code
        self.error_msg = 'Behavior Tree execute failed with error code %i' % self.error_code

        # saving context for replanning
        self.cur_pose = cur_pose
        self.cur_time = cur_time
        self.block = block
