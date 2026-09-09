#!/usr/bin/env python3
"""A/B 点标定工具: 让机械臂运动到指定 XYZ, 打印实际位姿与关节角.

用于确定可取的取物点/放置点坐标, 然后写入 pick_place_params.yaml。

用法:
  ros2 run robomaster_pick_place_sim calibrate_pose
  输入 "x y z" 回车 -> 规划并执行到该位置, 打印末端实际位姿
  输入空行 -> 打印当前位姿与关节角
  输入 q 退出
"""
import sys

import moveit_commander
from moveit_commander import MoveGroupCommander


def show(group):
    pose = group.get_current_pose().pose
    q = pose.orientation
    fx = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
    fy = 2.0 * (q.x * q.y + q.w * q.z)
    fz = 2.0 * (q.x * q.z - q.w * q.y)
    print(f'末端位置: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
    print(f'手指方向: ({fx:.3f}, {fy:.3f}, {fz:.3f})')
    print(f'关节角:   {[round(v, 3) for v in group.get_current_joint_values()]}')
    print()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    group = MoveGroupCommander('arm', wait_for_servers=60.0)
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)
    print('已连接 move_group。输入 "x y z" 移动, 空行显示当前位姿, q 退出')

    while True:
        try:
            line = sys.stdin.readline()
        except KeyboardInterrupt:
            break
        if not line:
            break
        line = line.strip()
        if line in ('q', 'quit', 'exit'):
            break
        if not line:
            show(group)
            continue
        try:
            x, y, z = map(float, line.split())
        except ValueError:
            print('格式: x y z')
            continue

        ok = group.set_position_target([x, y, z])
        plan = group.plan()  # (ok, traj, planning_time, error_code)
        if not ok or not plan[0]:
            print(f'不可达/无逆解: ({x}, {y}, {z})  error code={plan[3].code}')
            group.clear_pose_targets()
            continue
        if not group.execute(plan[1], wait=True):
            print('执行失败')
            continue
        group.clear_pose_targets()
        show(group)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
