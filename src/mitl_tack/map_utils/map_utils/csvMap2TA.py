"""
This file Created by i (i@)


Input:
    CSV Format Map (.csv);

Output: (5 files)
    1. map(x).ta; (Timed Automata of Map)
    2. map(x).npz; (Store Additional Map Region Infos)
    3. map(x).mitl; (Props)
    4. map(x)_origin.png/.pdf; (Map Visualize of .csv and target points)
    5. map(x)_decomp.png/.pdf; (Map Visualize of decomp regions)

Decompose Algorithm:
    Rectangle Cell Decomposition.
    (JS code from: https://kgithub.com/mikolalysenko/rectangle-decomposition)


 - CSV Format Map Path: /home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv
 - 'map(x).ta' File Path: /home/devel_ws/src/mitl_tack/map_utils/outputs/
 - 'map(x).npz' File Path: /home/devel_ws/src/mitl_tack/map_utils/outputs/
 - 'map(x).mitl' File Path: /home/devel_ws/src/mitl_tack/map_utils/outputs/
 - '.png/.pdf' File Path: /home/devel_ws/src/mitl_tack/map_utils/outputs/

"""

import argparse
import os
import execjs
import random
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.patches as patches


random.seed(12345)


def _find_region(strat_point: list, csv_map: np.ndarray) -> list:
    _region = []
    index_x, index_y = strat_point

    while True:
        if (index_x, index_y) == strat_point and len(_region) != 0:
            break

        _grid = csv_map[index_y:index_y+2, index_x: index_x+2]

        # Some fault cases(MAP ERROR)(2):
        #       0(y, x) 0   |   1(y, x) 1
        #       0       0   |   1       1
        if (_grid == [[0, 0], [0, 0]]).all() or (_grid == [[1, 1], [1, 1]]).all():
            print("MAP ERROR With Case: \'0000\' or \'1111\'!")
            return None

        # case1 & case 2: (move down & append, y += 1)
        #       1(y, x) 1
        #       1       0
        #   or
        #       0(y, x) 0
        #       1       0
        if (_grid == [[1, 1], [1, 0]]).all() or (_grid == [[0, 0], [1, 0]]).all():
            _region.append([index_x, index_y])
            index_y += 1
            continue

        # case3 & case4: (move up & append, y -= 1)
        #       0(y, x) 1
        #       1       1
        #   or
        #       0(y, x) 1
        #       0       0
        if (_grid == [[0, 1], [1, 1]]).all() or (_grid == [[0, 1], [0, 0]]).all():
            _region.append([index_x, index_y])
            index_y -= 1
            continue

        # case5 & case6: (move right & append, x += 1)
        #       1(y, x) 0
        #       1       1
        #   or
        #       0(y, x) 0
        #       0       1
        if (_grid == [[1, 0], [1, 1]]).all() or (_grid == [[0, 0], [0, 1]]).all():
            _region.append([index_x, index_y])
            index_x += 1
            continue

        # case7 & case8: (move left & append, x -= 1)
        #       1(y, x) 0
        #       0       0
        #   or
        #       1(y, x) 1
        #       0       1
        if (_grid == [[1, 0], [0, 0]]).all() or (_grid == [[1, 1], [0, 1]]).all():
            _region.append([index_x, index_y])
            index_x -= 1
            continue

        # case9: (move down, y += 1)
        #       1(y, x) 0
        #       1       0
        if (_grid == [[1, 0], [1, 0]]).all():
            index_y += 1
            continue

        # case10: (move right, x += 1)
        #       0(y, x) 0
        #       1       1
        if (_grid == [[0, 0], [1, 1]]).all():
            index_x += 1
            continue

        # case11: (move left, x -= 1)
        #       1(y, x) 1
        #       0       0
        if (_grid == [[1, 1], [0, 0]]).all():
            index_x -= 1
            continue

        # case12: (move up, y -= 1)
        #       0(y, x) 1
        #       0       1
        if (_grid == [[0, 1], [0, 1]]).all():
            index_y -= 1
            continue

        # case13: ()
        #       0(y, x) 1
        #       1       0

        # case14: ()
        #       1(y, x) 0
        #       0       1

    return _region


def _csv2regions(csv_file_path: str, max_target_region_num=3) -> list:
    """
    Input Value:
            csv_file_path: 
    Return Value List: 
            1. list(region): Regions of (wall + inside obstacle)
            2. list(_target_region): target Regions/ Target Piont (target regions/target point)
            ! NOTE: _target_region is not included in region!

    """
    csv_map = np.loadtxt(csv_file_path, dtype=int, delimiter=',')
    mapSize_y, mapSize_x = np.shape(csv_map)

    region = []
    _target_region = []

    # ---------------------------------------------------
    # region of the wall, find margin like a snake
    # ---------------------------------------------------

    # ! _wall -> anti-clockwise; reverse -> clockwise
    _wall = _find_region(strat_point=(0, 0), csv_map=csv_map)
    _wall.reverse()
    region.append(_wall)

    # ---------------------------------------------------
    # region of the inner obstacles, differ in start point.
    # ---------------------------------------------------
    # AND
    # ---------------------------------------------------
    # ADD Region of the target regions
    # CASE :
    #       0   0   0               0   0   0
    #       0   0   0      --->     0   2   0
    #       0   0   0               0   0   0
    #
    # @ parm: target_region_num
    #       num of the target regions. Default num three.
    #
    # @ return: target_regions
    #       list of target region
    # ---------------------------------------------------

    # var to record last target point,
    # and for the next target point, shouldn't to close to the last.
    # abs(last_x - x) > 2 or abs(last_y - y) > 2
    csv_map_deepCopy = deepcopy(csv_map)

    for x in range(1, mapSize_x - 1):
        for y in range(1, mapSize_y - 1):

            # ADD Region of the target regions.
            if max_target_region_num != 0 \
                    and np.array_equal(csv_map_deepCopy[y:y+3, x:x+3], np.zeros((3, 3), dtype=int)) \
                        and random.random() >= 0.5:

                # ! _target: counter-clockwise
                _target = [[x, y], [x, y+1], [x+1, y+1], [x+1, y]]
                csv_map_deepCopy[y:y+3, x:x+3] = 2
                # region.append(_target)
                _target_region.append(_target)
                max_target_region_num -= 1

            # check for repeat (inner obstacles)
            tag_repeat = 0
            for tmp in region:
                if [x, y] in tmp:
                    tag_repeat = 1
                    break
            if tag_repeat:
                continue

            _grid = csv_map[y:y+2, x:x+2]
            # cases do not '_find_region'
            if (_grid == [[0, 0], [0, 0]]).all() or \
                (_grid == [[1, 1], [1, 1]]).all() or \
                (_grid == [[0, 0], [1, 1]]).all() or \
                (_grid == [[1, 1], [0, 0]]).all() or \
                (_grid == [[1, 0], [1, 0]]).all() or \
                    (_grid == [[0, 1], [0, 1]]).all():
                continue

            # ! _inner.reverse(): counter clock wise;
            _inner = _find_region(strat_point=(x, y), csv_map=csv_map)
            _inner.reverse()
            region.append(_inner)

    return region, _target_region


def _js_decompose(region):
    """
    @ parm: region (borders of the map)

    Sample of region:
            region = [
                [[1, 1], [1, 2], [2, 2], [2, 1]],
                [[0, 0], [4, 0], [4, 4], [1, 4], [1, 3], [0, 3]]
            ]

    ! Note 1 : Walls in region must be Clockwise !!! Else Failure occure at 'decomposeRegion';
    ! Note 2 : Inners & Targets in region must be Counter-Clockwise !!! Else Result Error of 'decomposeRegion';
    """
    with open('/home/devel_ws/src/mitl_tack/map_utils/map_utils/decomp.js', 'r', encoding='UTF-8') as file:
        js_code = file.read()

    js_ctx = execjs.compile(js_code)

    regions = js_ctx.call("decomposeRegion", region)

    regions, ops = _optimize_region_decomp(regions=regions)

    # [print(i) for i in regions]
    while ops != 0:
        regions, ops = _optimize_region_decomp(regions=regions)

    return regions


def _optimize_region_decomp(regions):
    # todo: restrict region decomp len: width < 2: 1 or 
    tmp_res = []
    operate = 0

    for region in regions:
        width = abs(region[0][0] - region[1][0])
        hight = abs(region[0][1] - region[1][1])
        if width > 2 * hight :
            tmp_res.append([region[0], [region[0][0] + width // 2, region[1][1]]])
            tmp_res.append([[region[0][0] + width // 2, region[0][1]], region[1]])
            operate += 2

        elif hight > 2 * width:
            tmp_res.append([region[0], [region[1][0], region[0][1] + hight // 2]])
            tmp_res.append([[region[0][0], region[0][1] + hight // 2], region[1]])
            operate += 2
        else:
            tmp_res.append(region)
    return tmp_res, operate


def _find_connections(decomp_regions: list, target_regions: list) -> list:
    """
    Describe:
        Function to find the connection relations between: 
                1. Decomp_regions & Decomp_regions; 
                2. Decomp_regions & Target_regions; 

        Function to calculate the 'guard' of Decomp_regions & Target_regions; 

    Process:
        decomp_regions & target_regions -()-> decomp_regions_in_line & target_regions_in_line 
            -()-> match_line & find the connect relations

    @ parm: decomp_regions (list)
        decomp_regions from _js_decompose
    @ parm: target_regions (list)
        target_regions.

    @ return: (4 lists, 2 connections list + 2 guard list)
        1. conn_decomp_decomp (list): Connections between Decomp_regions & Decomp_regions; 
        2. conn_decomp_target (list): Connections between Decomp_regions & Target_regions;

        3. guard_decomp (list):
        4. guard_target (list):
    """

    def __sort_edge_of_region(region_min_max: list) -> list:
        """
            @ Input: region_min_max
                [x_min, x_max, y_min, y_max]

            Sort the lines as follow:

            ----|----------------> x
                |     3
                |   *---*
                | 0 |   | 2
                |   *---*
                |     1
                y

            0: [x_min, y_min, x_min, y_max]     1: [x_min, y_max, x_max, y_max]
            2: [x_max, y_max, x_max, y_min]     3: [x_max, y_min, x_min, y_min]
        """

        if len(region_min_max) != 4:
            return []

        x_min, x_max, y_min, y_max = region_min_max
        sorted_region = [
            [x_min, y_min, x_min, y_max], [x_min, y_max, x_max, y_max],
            [x_max, y_max, x_max, y_min], [x_max, y_min, x_min, y_min]
        ]
        return sorted_region

    def __decomp_region_toedges(decomp_regions) -> list:
        """
            Parse 'decomp_region' into line representation, and sort (__sort_line_of_region);

            'decomp_region': (represent in 2 Diagonal Points.)
                [[[x1, y1], [x2, y2]], ...] 

            'lines_of_region': (Line Representation. '[start_x, start_y, end_x, end_y]')
                [[[x1, y1, x2, y2], [x2, y2, x3, y3], ...], ...]
        """
        lines_of_decomp_region = []

        for _region in decomp_regions:
            x_min, x_max = min(_region[0][0], _region[1][0]), max(
                _region[0][0], _region[1][0])
            y_min, y_max = min(_region[0][1], _region[1][1]), max(
                _region[0][1], _region[1][1])

            lines_of_decomp_region.append(
                __sort_edge_of_region([x_min, x_max, y_min, y_max]))

        return lines_of_decomp_region

    def __target_region_toedges(target_regions) -> list:
        """
            Parse 'target_regions' into line representation, and sort (__sort_line_of_region);

            'target_region': (represent in 4 Points.)
                [[[x1, y1], [x2, y2], ...], ...] 

            'lines_of_region': (Line Representation. '[start_x, start_y, end_x, end_y]')
                [[[x1, y1, x2, y2], [x2, y2, x3, y3], ...], ...]
        """
        lines_of_target_region = []

        for _region in target_regions:

            axisX, axisY = [x[0] for x in _region], [x[1] for x in _region]

            x_min, x_max = min(axisX), max(axisX)
            y_min, y_max = min(axisY), max(axisY)

            lines_of_target_region.append(
                __sort_edge_of_region([x_min, x_max, y_min, y_max]))

        return lines_of_target_region

    def __is_edges_overlapping(lineA: list, lineB: list, isVertical: bool) -> bool:

        # line: [x1, y1, x2, y2]
        # for case 'Vertical', select the y axis,
        # else select the x axis.
        if isVertical:

            if lineA[0] != lineB[0]:
                return False

            lineA_min, lineA_max = min(
                lineA[1], lineA[-1]), max(lineA[1], lineA[-1])
            lineB_min, lineB_max = min(
                lineB[1], lineB[-1]), max(lineB[1], lineB[-1])
        else:

            if lineA[1] != lineB[1]:
                return False

            lineA_min, lineA_max = min(
                lineA[0], lineA[2]), max(lineA[0], lineA[2])
            lineB_min, lineB_max = min(
                lineB[0], lineB[2]), max(lineB[0], lineB[2])

        if lineA_min >= lineB_max or lineA_max <= lineB_min:
            return False
        return True

    def __find_connection_regions_with_regions(regionListA: list, regionListB: list) -> list:
        """
        Describe:
            Given two Region lists (edge form), return 
                1. (list): Region connection relations.
                2. (list): Distance if connected.

        @ 
        """
        conn_list = []

        for regionA, indexA in zip(regionListA, range(len(regionListA))):
            for regionB, indexB in zip(regionListB, range(len(regionListB))):

                # No Repeat
                if [indexA, indexB] in conn_list or [indexB, indexA] in conn_list:
                    continue

                # Four Cases of A & B Connected.

                # Case 1: regionA.Edge_0    <--->   regionB.Edge_2
                #   Or
                # Case 2: regionA.Edge_2    <--->   regionB.Edge_0
                # isVertical = True
                if __is_edges_overlapping(regionA[0], regionB[2], isVertical=True) or \
                        __is_edges_overlapping(regionA[2], regionB[0], isVertical=True):
                    conn_list.append([indexA, indexB])
                    continue

                # Case 3: regionA.Edge_1    <--->   regionB.Edge_3
                #   Or
                # Case 4: regionA.Edge_3    <--->   regionB.Edge_1
                # isVertical = False
                if __is_edges_overlapping(regionA[1], regionB[3], isVertical=False) or \
                        __is_edges_overlapping(regionA[3], regionB[1], isVertical=False):
                    conn_list.append([indexA, indexB])
                    continue

        return conn_list

    def __cal_guard(regions_inedge: list) -> list:
        _tmp_res = []

        for region in regions_inedge:
            guard = ((region[0][1] - region[0][3])**2 + (region[1][0] - region[1][2])**2) ** 0.5
            guard = int(guard * 2.5 + 1)
            # guard = (Length + width)  * meter/pixel / vel
            _tmp_res.append(guard)
        return _tmp_res

    # Logic of Function '_find_connections' Start Here:
    decomp_regions_inedge = __decomp_region_toedges(
        decomp_regions=decomp_regions)
    target_regions_inedge = __target_region_toedges(
        target_regions=target_regions)

    conn_decomp_decomp = __find_connection_regions_with_regions(
        regionListA=decomp_regions_inedge, regionListB=decomp_regions_inedge)
    conn_decomp_target = __find_connection_regions_with_regions(
        regionListA=decomp_regions_inedge, regionListB=target_regions_inedge)

    guard_decomp, guard_target = __cal_guard(
        decomp_regions_inedge), __cal_guard(target_regions_inedge)

    return conn_decomp_decomp, guard_decomp, \
        conn_decomp_target, guard_target


def _to_TA(ta_path: str, decomp_regions_len: int, target_regions_len: int,
           conn_decomp_decomp: list, conn_decomp_target: list,
           guard_decomp: list, guard_target: list) -> None:
    """
    @ parm: 
        ta_path (str): 

        decomp_regions_len (int): 

        target_regions_len (int): 

        conn_decomp_decomp (list): 

        conn_decomp_target (list): 

        guard_decomp (list): 

        guard_target (list): 

    """

    decomp_region_nameSpace = ["decomp_region_" +
                               str(i) for i in range(decomp_regions_len)]
    target_region_nameSpace = ["target_region_" +
                               str(i) for i in range(target_regions_len)]

    # Writing to TA file, path: ta_path
    ta_file = open(ta_path, 'w')

    # Adding Start Info of the '.ta' file.
    ta_file.write("// ------------------------------------------------------------\n"
                  "// Robot Environment (Small Map)\n"
                  "//\n"
                  "// automatically generated by python script src/mitl_tack/map_utils/map_utils/csvMap2TA.py \n"
                  "// i <i@> \n"
                  "// Tue Sep 24 17:16:18 2022 \n"
                  "// ------------------------------------------------------------ \n\n")
    ta_file.write("clock x1;\n\n"
                  "process p1 {\n"
                  "\n"
                  "  state ")

    # Adding States of the TA to '.ta' file.
    for i in range(decomp_regions_len):
        if i == 0:
            ta_file.write(
                decomp_region_nameSpace[i] + " {x1<=" + str(guard_decomp[i] * 2) + "}")
        else:
            ta_file.write(
                ',\n\t\t' + decomp_region_nameSpace[i] + " {x1<=" + str(guard_decomp[i] * 2) + "}")
    for i in range(target_regions_len):
        ta_file.write(
            ',\n\t\t' + target_region_nameSpace[i] + " {x1<=" + str(guard_target[i] * 2) + "}")

    ta_file.write(";\n\n  init %s;\n\n" % (decomp_region_nameSpace[0]))

    # Adding Trans relations of the TA to '.ta' file.
    ta_file.write("  trans   ")

    for i in range(len(conn_decomp_decomp)):
        trans = conn_decomp_decomp[i]
        # { guard x1>3; assign x1:=0; }
        attach_0 = " { guard x1>" + str(guard_decomp[trans[0]]) + "; assign x1:=0; }"
        attach_1 = " { guard x1>" + str(guard_decomp[trans[1]]) + "; assign x1:=0; }"

        if i == 0:
            ta_file.write(
                decomp_region_nameSpace[trans[0]] + " -> " + decomp_region_nameSpace[trans[1]] + attach_0)
            ta_file.write(
                ",\n\t\t" + decomp_region_nameSpace[trans[1]] + " -> " + decomp_region_nameSpace[trans[0]] + attach_1)

        else:
            ta_file.write(
                ",\n\t\t" + decomp_region_nameSpace[trans[0]] + " -> " + decomp_region_nameSpace[trans[1]] + attach_0)
            ta_file.write(
                ",\n\t\t" + decomp_region_nameSpace[trans[1]] + " -> " + decomp_region_nameSpace[trans[0]] + attach_1)

    for i in range(len(conn_decomp_target)):
        trans = conn_decomp_target[i]
        # { guard x1>3; assign x1:=0; }
        attach_0 = " { guard x1>" + str(guard_decomp[trans[0]]) + "; assign x1:=0; }"
        attach_1 = " { guard x1>" + str(guard_target[trans[1]]) + "; assign x1:=0; }"

        ta_file.write(
            ",\n\n\t\t" + decomp_region_nameSpace[trans[0]] + " -> " + target_region_nameSpace[trans[1]] + attach_0)
        ta_file.write(
            ",\n\t\t" + target_region_nameSpace[trans[1]] + " -> " + decomp_region_nameSpace[trans[0]] + attach_1)

    ta_file.write(";\n\n")

    ta_file.write("}\n\nsystem p1;\n")
    ta_file.close()
    return


def _to_MITL(mitl_path: str, time_limit: int, target_regions_len: int) -> None:
    formula = 'G_i+ 0 (F_ee 0 %s (p1_target_region_0))' % (str(time_limit))
    for i in range(1, target_regions_len):
        tmp = 'G_i+ 0 (F_ee 0 %s (p1_target_region_%s))' % (str(time_limit), str(i))
        formula = '&& (%s) (%s)' % (formula, tmp)

    with open(mitl_path, 'w') as file:
        file.write("! (%s)" % formula)
    return


class npzNode(object):
    """

    """

    def __init__(self, regions: list, target_regions: list, region_decomp: list,
                 conn_decomp_decomp: list, guard_decomp: list,
                 conn_decomp_target: list, guard_target: list) -> None:

        self.regions = regions
        self.target_regions = target_regions
        self.region_decomp = region_decomp
        self.conn_decomp_decomp = conn_decomp_decomp
        self.guard_decomp = guard_decomp
        self.conn_decomp_target = conn_decomp_target
        self.guard_target = guard_target


def _to_NPZ(result_path: str, all_data: npzNode) -> None:

    np.savez(
        result_path,

        regions=all_data.regions,
        target_regions=all_data.target_regions,
        region_decomp=all_data.region_decomp,
        conn_decomp_decomp=all_data.conn_decomp_decomp,
        guard_decomp=all_data.guard_decomp,
        conn_decomp_target=all_data.conn_decomp_target,
        guard_target=all_data.guard_target
    )

    return


def _plot_map(csv_map_file: str, target_regions: list, decomp_regions: list, result_dir: str) -> None:
    """

        Saving to 2 files, default figure type is '.png':
            1. result_dir + map(x)_origin + 'figure type';
            2. result_dir + map(x)_decomp + 'figure type';
    """
    figure_type = '.pdf'
    # example of csv_map_file:
    #       /home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv
    map_index = os.path.split(csv_map_file)[-1].split('.')[0]
    origin_figure_path = os.path.join(
        result_dir, map_index + '_origin' + figure_type)
    decomp_figure_path = os.path.join(
        result_dir, map_index + '_decomp' + figure_type)

    def __axis_trans_csv_map(x: int, y: int) -> list:
        return [x, -y - 1]

    def __axis_trans_noWall_coord(x: int, y: int) -> list:
        return [x + 1, -y - 1]

    csv_map = np.loadtxt(csv_map_file, dtype=int, delimiter=',')
    mapSizeY, mapSizeX = np.shape(csv_map)
    grid_size = 1.

    ax = plt.subplot()
    ax.axis([0, mapSizeX, -mapSizeY, 0])
    plt.gcf().set_size_inches(mapSizeX, mapSizeY)

    # plot basic maps.
    for x in range(mapSizeX):
        for y in range(mapSizeY):

            if csv_map[y][x] == 1:
                ax.add_patch(
                    patches.Rectangle(
                        __axis_trans_csv_map(x, y), grid_size, grid_size,
                        facecolor='gray', alpha=.9
                    )
                )

    # plot target Region.
    for target, index in zip(target_regions, range(len(target_regions))):
        # print(target)
        x, y = target[0]
        # y + 1 as trans left-upper point (x, y) to left_down point (x, y + 1)
        x, y = __axis_trans_noWall_coord(x, y + 1)

        plt.text(x + grid_size / 2, y + grid_size / 2, str(index))

        ax.add_patch(
            patches.Rectangle(
                (x, y), grid_size, grid_size,
                facecolor='green', alpha=.9
            )
        )

    plt.savefig(origin_figure_path)

    # Plot Decomp Regions.
    for decomp, index in zip(decomp_regions, range(len(decomp_regions))):
        # print(decomp)
        x, y = decomp[0]
        x, y = __axis_trans_noWall_coord(x, y)
        width = abs(decomp[0][0] - decomp[1][0])
        hight = abs(decomp[0][1] - decomp[1][1])

        plt.text(x + width / 2, y - hight / 2, str(index))

        ax.add_patch(
            patches.Rectangle(
                (x, y), width, -hight,
                edgecolor='red', linewidth=1., capstyle='round', fill=False
            )
        )

    plt.savefig(decomp_figure_path)
    # plt.show()

    return


def decompose_single_csv_map(csv_map_file: str, result_dir: str) -> None:
    """
    Describe:
        Function for decompose single csv map file.
        And Plot:
            1. The origion_map with target regions. (csv_map + target_regions -> 'mapX_origion.png/.pdf')
            2. The decomposed map. (csv_map + region_decomp -> 'mapX_decomp.png/.pdf')

    Process:
        CSV Map -(_csv2region)-> regions & target_regions -(_js_decompose)-> region_decomp
            --> Cal Connection Relations --> Create TA / Prop & Saving to files

    @ parm:
        csv_map_file: single csv map file path.

    @ return: None (With .ta & .mitl & .npz & .png/.pdf files saved to resilt directly)
    """

    regions, target_regions = _csv2regions(csv_file_path=csv_map_file)
    # demo region for decomposition
    # region_demo = [
    #     [[1, 1], [1, 2], [2, 2], [2, 1]],
    #     [[0, 0], [4, 0], [4, 4], [1, 4], [1, 3], [0, 3]]]
    # region_map1 = [
    #     [[1, 1], [1, 2], [2, 2], [2, 1]],
    #     [[0, 0], [0, 3], [3, 3], [3, 5], [0, 5], [0, 8], [6, 8], [6, 0]]
    #     ]
    region_decomp = _js_decompose(region=regions + target_regions)

    conn_decomp_decomp, guard_decomp, conn_decomp_target, guard_target = _find_connections(
        decomp_regions=region_decomp, target_regions=target_regions)

    # -------------------------------------------------------------------
    # Plotting.
    # -------------------------------------------------------------------
    _plot_map(csv_map_file=csv_map_file,
              target_regions=target_regions, decomp_regions=region_decomp,
              result_dir=result_dir)

    # -------------------------------------------------------------------
    # Saving to files.
    # -------------------------------------------------------------------

    # example of csv_map_file:
    #       /home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv
    # Example of Map Index: 'max1'
    map_index = os.path.split(csv_map_file)[-1].split('.')[0]

    _to_TA(
        ta_path=os.path.join(result_dir, map_index + '.ta'),
        decomp_regions_len=len(region_decomp),
        target_regions_len=len(target_regions),
        conn_decomp_decomp=conn_decomp_decomp,
        conn_decomp_target=conn_decomp_target,
        guard_decomp=guard_decomp,
        guard_target=guard_target
    )

    _to_MITL(
        mitl_path=os.path.join(result_dir, map_index + '.mitli'),
        time_limit=150,
        target_regions_len=len(target_regions)
    )

    storage_npzNode = npzNode(
        regions=regions,
        target_regions=target_regions,
        region_decomp=region_decomp,
        conn_decomp_decomp=conn_decomp_decomp,
        conn_decomp_target=conn_decomp_target,
        guard_decomp=guard_decomp,
        guard_target=guard_target
    )
    _to_NPZ(result_path=os.path.join(
        result_dir, map_index + '.npz'), all_data=storage_npzNode)

    # print(conn_decomp_decomp)
    # print()
    # print(conn_decomp_target)

    return


def decompose_csv_maps(csv_map_dir: str, result_dir: str) -> None:

    csv_map_files = [os.path.join(csv_map_dir, i)
                     for i in os.listdir(csv_map_dir) if i[-4:] == '.csv']

    for _csv_map_file in csv_map_files:
        print('Proccessing CSV Map File: ' + _csv_map_file)
        decompose_single_csv_map(
            csv_map_file=_csv_map_file, result_dir=result_dir)

    return


if __name__ == '__main__':
    # csv_map_dir = '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/'

    # csv_map_file = '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv'

    # decompose_single_csv_map(csv_map_file=csv_map_file, result_dir=const_result_dir)

    # region, target_region = _csv2regions(csv_file_path=csv_map_file)
    # print(region, '\n', target_region)

    """
        Agrument Parse:

        @ parm:
            -f [csv_map_file] : csv map file path, default map file: None.
                (x) '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv'

            -d [csv_maps_dir] : csv map files dir, default map file: None.
                (x) '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/'

            -h : usage

        @ Output Dir (Cannot Config): 
            '/home/devel_ws/src/mitl_tack/map_utils/outputs/'

    """

    def _usages():
        print('csvMap2TA.py: Trans CSV form map(s) to Timed Automata & MITL Props.')
        print('Usage: ')
        print(
            '     -f [csv_map_file] : csv map file path, MUST From Parameter Input.')
        print()
        print(
            '     -d [csv_maps_dir] : csv map files dir, MUST From Parameter Input.')
        print()
        print('     -h : helps')

    parser = argparse.ArgumentParser("csvMap2TA.py", add_help=False)
    parser.add_argument('-f', '--csv_map_file')
    parser.add_argument('-d', '--csv_maps_dir')
    parser.add_argument('-h', '--help')
    args = parser.parse_args()

    const_result_dir = '/home/devel_ws/src/mitl_tack/map_utils/outputs/'

    if args.csv_map_file:
        print('Decompose single csv map at ' + args.csv_map_file)

        decompose_single_csv_map(
            csv_map_file=args.csv_map_file, result_dir=const_result_dir)

    elif args.csv_maps_dir:
        print('Decompose csv maps at ' + args.csv_maps_dir)

        decompose_csv_maps(csv_map_dir=args.csv_maps_dir,
                           result_dir=const_result_dir)

    else:
        _usages()

    # todo: 20220926 1. Add Notes 2. Complete arg parse ('-f', '-d', '-h')
    # ! ERROR: Some Complex CSV Maps Decompose Out Error. (decompose_csv_maps)
