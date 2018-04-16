#!/usr/bin/env python
# -*- coding: utf-8 -*-

def update_assignments(assignments, target_id_list, key,
        goalie_id, assignment_type=None, closest_role=None):
    # IDが存在しないRoleをNoneにする
    id_list = target_id_list[:] # Deep copy
    for role, robot_id in assignments.items():
        if not robot_id in id_list:
            assignments[role] = None

    # IDsからすでにRoleが登録されてるIDを取り除く
    for robot_id in assignments.values():
        if robot_id in id_list:
            id_list.remove(robot_id)

    # Role_0にGoalie_IDを登録する
    if goalie_id in id_list:
        id_list.remove(goalie_id)
        assignments[key+'0'] = goalie_id

    # 残ったIDを順番にRoleに登録する
    for role, robot_id in assignments.items():
        if id_list and role != key+'0' and robot_id is None:
            assignments[role] = id_list.pop(0)

    # IDが登録されてないRoleは末尾から詰める
    target_i = 1
    replace_i = 5
    while replace_i - target_i > 0:
        while replace_i > 2:
            if assignments[key + str(replace_i)] is not None:
                break
            replace_i -= 1

        target_role = key + str(target_i)
        if assignments[target_role] is None:
            replace_role = key + str(replace_i)
            replace_ID = assignments[replace_role]
            assignments[target_role] = replace_ID
            assignments[replace_role] = None

        target_i += 1

    if assignment_type == 'CLOSEST_BALL':
        if closest_role:
            old_id = assignments['Role_1']
            assignments['Role_1'] = assignments[closest_role]
            assignments[closest_role] = old_id

    return assignments


