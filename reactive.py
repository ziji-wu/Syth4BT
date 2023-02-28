import re
import os


def cmd_ros():
    result = os.popen('ls')
    print()
    context = result.read()
    # for line in context.splitlines():
    #     print(line)
    # result.close()
    print(context)
    return


def new_formula(cur_time: int, fp_time: int) -> None:
    # case1: F_ii lt ut or F_ee lt ut
    #       or G_ii lt ut or G_ee lt ut
    # case2: F_i+ lt or F_e+ lt or G_i+ lt or G_e+ lt

    formula = '! (&& (F_ii 0 120 (p1_target_region_1)) (&& (F_i+ 120 (p1_target_region_0)) (F_i+ 0 (p1_target_region_2))))'

    cost = cur_time + fp_time

    num_pattern = re.compile(r'\d+')
    case1_pattern = re.compile(r'(F|G)\_(ii|ee) \d+ \d+')
    case2_pattern = re.compile(r'(F|G)\_(i\+|e\+) \d+')

    for match in case1_pattern.finditer(formula):
        match = match.group()
        times = [int(i) for i in num_pattern.findall(match)]
        times = [max(0, i - cost) for i in times]

        if times[1] == 0:
            # todo: task failed, raise error
            pass
        else:
            formula = re.sub(match,
                             match[:5] + str(times[0]) + ' ' + str(times[1]), formula)

    for match in case2_pattern.finditer(formula):
        match = match.group()
        times = [int(i) for i in num_pattern.findall(match)]
        times = [max(0, i - cost) for i in times]

        match_re = re.sub(r'\+', '\+', match)
        formula = re.sub(match_re,
                         match[:5] + str(times[0]), formula)
        
    print(formula)

    return

def stop_planning():
    """
        Stop Docker
    """
    res = os.popen('sudo docker ps')
    ctx = res.read().split()
    
    if len(ctx) > 8:
        os.system('sudo docker stop ' + ctx[8])



stop_planning()